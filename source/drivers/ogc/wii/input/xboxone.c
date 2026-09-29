#ifdef HW_RVL

/*
 * Wired Xbox One / Xbox One S input for Snes9x GX.
 *
 * This driver talks directly to IOS58 /dev/usb/ven. Device discovery follows
 * the documented USB_VEN event model: GetDeviceChange is ALWAYS asynchronous.
 * A synchronous GetDeviceChange can sleep until the next USB topology change,
 * which would stall the emulator when no controller is connected.
 *
 * Lifecycle used here:
 *   Open handle -> async GetDeviceChange -> Attach matching interface
 *   -> Resume -> GetDeviceInfo -> AttachFinish -> queue interrupt IN
 *   -> send Xbox GIP init.
 *
 * Xbox One S 045e:02ea uses interface FF/47/D0. Init packets and sequencing
 * follow Linux xpad and the working WiiredX implementation. All IOS-visible
 * buffers live in MEM2 and are 32-byte aligned; the first IntrTransfer vector
 * is always 64 bytes.
 */

#include <gccore.h>
#include <ogc/ipc.h>
#include <ogc/system.h>
#include <stdio.h>
#include <string.h>

#define MICROSOFT_VID          0x045e
#define XBOX_ONE_S_PID         0x02ea

#define GIP_IF_CLASS           0xff
#define GIP_IF_SUBCLASS        0x47
#define GIP_IF_PROTOCOL        0xd0

#define GIP_CMD_INPUT          0x20
#define GIP_CMD_VIRTUAL_KEY    0x07

#define VEN_IOCTL_GETVERSION       0
#define VEN_IOCTL_GETDEVICECHANGE  1
#define VEN_IOCTL_GETDEVPARAMS     3
#define VEN_IOCTL_ATTACH           4
#define VEN_IOCTL_RELEASE          5
#define VEN_IOCTL_ATTACHFINISH     6
#define VEN_IOCTL_SUSPEND_RESUME  16
#define VEN_IOCTL_CANCELENDPOINT  17
#define VEN_IOCTL_INTRMSG         19

#define MAX_REPORT_SIZE        64
#define MAX_USB_DEVICES        32
#define READ_RETRY_FRAMES       3
#define STICK_THRESHOLD     16384
#define TRIGGER_THRESHOLD      128
#define USB_NAK_RC          (-7005)

/* One permanent Arena2 allocation. Every address below is 32-byte aligned. */
#define VEN_BLOCK_SIZE       0x500
#define OFF_LIST             0x000 /* 0x180 */
#define OFF_CMD              0x180 /* 0x020 */
#define OFF_INFO             0x1a0 /* 0x0c0 */
#define OFF_IN_HEADER        0x260 /* 0x040 */
#define OFF_IN_DATA          0x2a0 /* 0x040 */
#define OFF_OUT_HEADER       0x2e0 /* 0x040 */
#define OFF_OUT_DATA         0x320 /* 0x040 */
#define OFF_IN_VEC           0x360 /* 0x020 reserved */
#define OFF_OUT_VEC          0x380 /* 0x020 reserved */
#define OFF_VERSION          0x3a0 /* 0x020 */

static const char venPath[] ATTRIBUTE_ALIGN(32) = "/dev/usb/ven";

static bool initialized = false;
static bool active = false;

static s32 venFd = -1;
static int venHandleId = -1;
static s32 deviceId = -1;
static bool deviceOwned = false;
static u8 endpointIn = 0;
static u8 endpointOut = 0;
static u16 reportLength = MAX_REPORT_SIZE;
static u8 outSequence = 1;

static volatile u32 heldButtons = 0;
static volatile bool guidePressed = false;
static volatile u32 rxPackets = 0;
static volatile u8 lastRxCommand = 0;

/* Lifecycle / transport diagnostics. */
static s32 rcVersion = 999;
static volatile s32 rcList = 999;
static s32 rcAttach = 999;
static s32 rcAttachFinish = 999;
static s32 rcResume = 999;
static s32 rcDeviceInfo = 999;
static s32 rcReadSubmit = 999;
static s32 rcPower = 999;
static s32 rcSpecial = 999;
static volatile s32 rcReadComplete = 999;
static char statusText[192] = "not found";

/* Persistent MEM2 buffers. */
static u8 *venBlock = NULL;
static u8 *deviceList = NULL;
static u8 *cmdBuffer = NULL;
static u8 *deviceInfo = NULL;
static u8 *inHeader = NULL;
static u8 *inData = NULL;
static u8 *outHeader = NULL;
static u8 *outData = NULL;
static u8 *versionBuffer = NULL;
static ioctlv *inVec = NULL;
static ioctlv *outVec = NULL;

/* Device-change hook state. Callback does no protocol work. */
static volatile bool listInFlight = false;
static volatile bool listReady = false;

/* Async read state. Callback only records state; all protocol work is main-loop. */
static volatile bool readInFlight = false;
static volatile bool readCompleted = false;
static int readRetryCountdown = 0;

static bool allocateVenBuffers(void)
{
    if (venBlock)
        return true;

    venBlock = (u8 *)SYS_AllocArenaMem2Lo(VEN_BLOCK_SIZE, 32);
    if (!venBlock)
        return false;

    memset(venBlock, 0, VEN_BLOCK_SIZE);
    deviceList    = venBlock + OFF_LIST;
    cmdBuffer     = venBlock + OFF_CMD;
    deviceInfo    = venBlock + OFF_INFO;
    inHeader      = venBlock + OFF_IN_HEADER;
    inData        = venBlock + OFF_IN_DATA;
    outHeader     = venBlock + OFF_OUT_HEADER;
    outData       = venBlock + OFF_OUT_DATA;
    inVec         = (ioctlv *)(venBlock + OFF_IN_VEC);
    outVec        = (ioctlv *)(venBlock + OFF_OUT_VEC);
    versionBuffer = venBlock + OFF_VERSION;
    return true;
}

static void setCmdDevice(s32 id)
{
    memset(cmdBuffer, 0, 0x20);
    *(s32 *)(cmdBuffer + 0) = id;
}

static u8 entryInterfaceNumber(const usb_device_entry *entry)
{
    return (u8)((entry->token >> 8) & 0xff);
}

static s16 readS16LE(const u8 *p)
{
    return (s16)((u16)p[0] | ((u16)p[1] << 8));
}

static u16 readU16LE(const u8 *p)
{
    return (u16)p[0] | ((u16)p[1] << 8);
}

static s32 deviceChangeCallback(s32 result, void *userdata)
{
    (void)userdata;
    rcList = result;
    listInFlight = false;
    listReady = true;
    return 0;
}

static s32 readCallback(s32 result, void *userdata)
{
    (void)userdata;
    rcReadComplete = result;
    readInFlight = false;
    readCompleted = true;
    return 0;
}

static void buildTransferHeader(u8 *header, u8 *data, u16 len, u8 endpoint)
{
    memset(header, 0, 64);
    *(s32 *)(header + 0) = deviceId;
    *(void **)(header + 8) = data;
    *(u16 *)(header + 12) = len;
    *(u8 *)(header + 14) = endpoint;
}

static s32 queueInterruptIn(void)
{
    if (venFd < 0 || deviceId == -1 || !endpointIn || readInFlight)
        return IPC_EINVAL;

    memset(inData, 0, MAX_REPORT_SIZE);
    buildTransferHeader(inHeader, inData, reportLength, endpointIn);

    inVec[0].data = inHeader;
    inVec[0].len = 64;
    inVec[1].data = inData;
    inVec[1].len = reportLength;

    s32 rc = IOS_IoctlvAsync(venFd, VEN_IOCTL_INTRMSG, 1, 1,
                             inVec, readCallback, NULL);
    rcReadSubmit = rc;
    if (rc >= 0)
        readInFlight = true;
    return rc;
}

static s32 sendInterruptOut(const u8 *packet, u16 len, int seqOverride)
{
    if (venFd < 0 || deviceId == -1 || !endpointOut || !packet || !len)
        return IPC_EINVAL;

    memset(outData, 0, MAX_REPORT_SIZE);
    memcpy(outData, packet, len);

    if (seqOverride >= 0) {
        outData[2] = (u8)seqOverride;
    } else {
        outData[2] = outSequence++;
        if (outSequence == 0)
            outSequence = 1;
    }

    buildTransferHeader(outHeader, outData, len, endpointOut);
    outVec[0].data = outHeader;
    outVec[0].len = 64;
    outVec[1].data = outData;
    outVec[1].len = len;

    return IOS_Ioctlv(venFd, VEN_IOCTL_INTRMSG, 2, 0, outVec);
}

static void parseXboxOneReport(const u8 *d, u16 len)
{
    if (!d || len < 5)
        return;

    lastRxCommand = d[0];
    rxPackets++;

    if (d[0] == GIP_CMD_VIRTUAL_KEY) {
        guidePressed = (d[4] & 0x01) != 0;
        if (d[1] & 0x10) {
            static const u8 ack[] = {
                0x01, 0x20, 0x00, 0x09, 0x00, 0x07, 0x20,
                0x02, 0x00, 0x00, 0x00, 0x00, 0x00
            };
            sendInterruptOut(ack, sizeof(ack), d[2]);
        }
        return;
    }

    if (d[0] != GIP_CMD_INPUT || len < 18)
        return;

    u32 buttons = 0;

    if (d[4] & 0x04) buttons |= PAD_BUTTON_START;
    if (d[4] & 0x08) buttons |= PAD_TRIGGER_Z;
    if (d[4] & 0x10) buttons |= PAD_BUTTON_B;
    if (d[4] & 0x20) buttons |= PAD_BUTTON_A;
    if (d[4] & 0x40) buttons |= PAD_BUTTON_Y;
    if (d[4] & 0x80) buttons |= PAD_BUTTON_X;

    if (d[5] & 0x01) buttons |= PAD_BUTTON_UP;
    if (d[5] & 0x02) buttons |= PAD_BUTTON_DOWN;
    if (d[5] & 0x04) buttons |= PAD_BUTTON_LEFT;
    if (d[5] & 0x08) buttons |= PAD_BUTTON_RIGHT;
    if (d[5] & 0x10) buttons |= PAD_TRIGGER_L;
    if (d[5] & 0x20) buttons |= PAD_TRIGGER_R;

    if ((readU16LE(&d[6]) >> 2) > TRIGGER_THRESHOLD)
        buttons |= PAD_TRIGGER_L;
    if ((readU16LE(&d[8]) >> 2) > TRIGGER_THRESHOLD)
        buttons |= PAD_TRIGGER_R;

    s16 lx = readS16LE(&d[10]);
    s16 ly = (s16)-readS16LE(&d[12]);
    s16 rx = readS16LE(&d[14]);
    s16 ry = (s16)-readS16LE(&d[16]);

    if (ly >  STICK_THRESHOLD) buttons |= PAD_BUTTON_UP;
    if (ly < -STICK_THRESHOLD) buttons |= PAD_BUTTON_DOWN;
    if (lx < -STICK_THRESHOLD) buttons |= PAD_BUTTON_LEFT;
    if (lx >  STICK_THRESHOLD) buttons |= PAD_BUTTON_RIGHT;
    if (ry >  STICK_THRESHOLD) buttons |= PAD_BUTTON_UP;
    if (ry < -STICK_THRESHOLD) buttons |= PAD_BUTTON_DOWN;
    if (rx < -STICK_THRESHOLD) buttons |= PAD_BUTTON_LEFT;
    if (rx >  STICK_THRESHOLD) buttons |= PAD_BUTTON_RIGHT;

    heldButtons = buttons;
}

static bool parseGipInterface(void)
{
    const u8 *iface = deviceInfo + 52;
    if (iface[0] < 9 || iface[5] != GIP_IF_CLASS ||
        iface[6] != GIP_IF_SUBCLASS || iface[7] != GIP_IF_PROTOCOL)
        return false;

    endpointIn = 0;
    endpointOut = 0;
    reportLength = MAX_REPORT_SIZE;

    u8 numEndpoints = iface[4];
    if (numEndpoints > 8)
        numEndpoints = 8;

    for (u8 i = 0; i < numEndpoints; ++i) {
        const u8 *ep = deviceInfo + 64 + (i * 8);
        if (ep[0] < 7 || (ep[3] & 0x03) != USB_ENDPOINT_INTERRUPT)
            continue;

        u8 address = ep[2];
        u16 maxPacket = (u16)ep[4] | ((u16)ep[5] << 8);
        if (address & USB_ENDPOINT_IN) {
            if (!endpointIn) {
                endpointIn = address;
                if (maxPacket && maxPacket < reportLength)
                    reportLength = maxPacket;
            }
        } else if (!endpointOut) {
            endpointOut = address;
        }
    }

    return endpointIn != 0 && endpointOut != 0;
}

static void releaseCurrentDevice(void)
{
    if (venFd < 0 || deviceId == -1 || !deviceOwned || !cmdBuffer)
        return;

    setCmdDevice(deviceId);
    IOS_Ioctl(venFd, VEN_IOCTL_RELEASE, cmdBuffer, 0x20, NULL, 0);
    deviceOwned = false;
    deviceId = -1;
}

static void closeVenHandle(void)
{
    if (venFd >= 0)
        IOS_Close(venFd);
    venFd = -1;
    venHandleId = -1;
    listInFlight = false;
    listReady = false;
}

static bool openVenHandle(void)
{
    if (venFd >= 0)
        return true;

    /* libogc owns handle id 0; USB_VEN supports up to 16 independent IDs. */
    for (int handle = 1; handle < 16; ++handle) {
        s32 fd = IOS_Open(venPath, (u32)handle);
        if (fd >= 0) {
            venFd = fd;
            venHandleId = handle;
            return true;
        }
    }
    return false;
}

static s32 armDeviceChange(void)
{
    if (venFd < 0 || listInFlight)
        return IPC_EINVAL;

    memset(deviceList, 0, 0x180);
    listReady = false;
    s32 rc = IOS_IoctlAsync(venFd, VEN_IOCTL_GETDEVICECHANGE,
                            NULL, 0, deviceList, 0x180,
                            deviceChangeCallback, NULL);
    if (rc >= 0)
        listInFlight = true;
    else
        rcList = rc;
    return rc;
}

static bool initializeVenWatch(void)
{
    if (!allocateVenBuffers() || !openVenHandle()) {
        snprintf(statusText, sizeof(statusText), "not found VEN open");
        return false;
    }

    memset(versionBuffer, 0, 0x20);
    rcVersion = IOS_Ioctl(venFd, VEN_IOCTL_GETVERSION,
                          NULL, 0, versionBuffer, 0x20);
    if (rcVersion < 0 || *(u32 *)versionBuffer != 0x00050001) {
        snprintf(statusText, sizeof(statusText), "not found VEN v:%d", rcVersion);
        closeVenHandle();
        return false;
    }

    s32 rc = armDeviceChange();
    if (rc < 0) {
        snprintf(statusText, sizeof(statusText), "not found VEN hook:%d", rc);
        closeVenHandle();
        return false;
    }

    snprintf(statusText, sizeof(statusText), "not found VEN wait h:%d v:%d",
             venHandleId, rcVersion);
    return true;
}

static void finishChangeAndRearm(void)
{
    rcAttachFinish = IOS_Ioctl(venFd, VEN_IOCTL_ATTACHFINISH,
                               NULL, 0, NULL, 0);
    if (!active && venFd >= 0)
        armDeviceChange();
}

static bool connectFromReadyList(void)
{
    if (!listReady || venFd < 0)
        return false;

    listReady = false;
    s32 countResult = rcList;
    if (countResult < 0) {
        snprintf(statusText, sizeof(statusText), "not found VEN list:%d", countResult);
        closeVenHandle();
        return false;
    }

    usb_device_entry *entries = (usb_device_entry *)deviceList;
    int count = countResult;
    if (count > MAX_USB_DEVICES)
        count = MAX_USB_DEVICES;

    deviceId = -1;
    for (int i = 0; i < count; ++i) {
        if (entries[i].vid == MICROSOFT_VID &&
            entries[i].pid == XBOX_ONE_S_PID &&
            entryInterfaceNumber(&entries[i]) == 0) {
            deviceId = entries[i].device_id;
            break;
        }
    }

    if (deviceId == -1) {
        rcAttach = 999;
        finishChangeAndRearm();
        snprintf(statusText, sizeof(statusText),
                 "not found VEN h:%d list:%d", venHandleId, countResult);
        return false;
    }

    rcAttach = rcAttachFinish = rcResume = rcDeviceInfo = 999;
    rcReadSubmit = rcPower = rcSpecial = rcReadComplete = 999;
    heldButtons = 0;
    rxPackets = 0;
    lastRxCommand = 0;
    outSequence = 1;

    /* GetDeviceChange owns the manager lock for this handle. The ownership
     * sequence used by working USBv5 controller backends is:
     * Attach -> Resume -> GetDeviceInfo -> AttachFinish. */
    setCmdDevice(deviceId);
    rcAttach = IOS_Ioctl(venFd, VEN_IOCTL_ATTACH,
                         cmdBuffer, 0x20, NULL, 0);
    if (rcAttach < 0) {
        deviceId = -1;
        finishChangeAndRearm();
        snprintf(statusText, sizeof(statusText),
                 "not found VEN attach:%d", rcAttach);
        return false;
    }
    deviceOwned = true;

    setCmdDevice(deviceId);
    ((s32 *)cmdBuffer)[2] = 1; /* state byte 11 on big-endian PPC */
    rcResume = IOS_Ioctl(venFd, VEN_IOCTL_SUSPEND_RESUME,
                         cmdBuffer, 0x20, NULL, 0);
    if (rcResume < 0) {
        releaseCurrentDevice();
        rcAttachFinish = IOS_Ioctl(venFd, VEN_IOCTL_ATTACHFINISH,
                                   NULL, 0, NULL, 0);
        armDeviceChange();
        snprintf(statusText, sizeof(statusText),
                 "not found VEN a:%d r:%d f:%d",
                 rcAttach, rcResume, rcAttachFinish);
        return false;
    }

    setCmdDevice(deviceId);
    memset(deviceInfo, 0, 0xc0);
    rcDeviceInfo = IOS_Ioctl(venFd, VEN_IOCTL_GETDEVPARAMS,
                             cmdBuffer, 0x20, deviceInfo, 0xc0);
    bool gipOk = (rcDeviceInfo >= 0) && parseGipInterface();
    if (!gipOk) {
        releaseCurrentDevice();
        rcAttachFinish = IOS_Ioctl(venFd, VEN_IOCTL_ATTACHFINISH,
                                   NULL, 0, NULL, 0);
        armDeviceChange();
        snprintf(statusText, sizeof(statusText),
                 "not found VEN a:%d r:%d d:%d g:%d f:%d",
                 rcAttach, rcResume, rcDeviceInfo, gipOk ? 1 : 0,
                 rcAttachFinish);
        return false;
    }

    rcAttachFinish = IOS_Ioctl(venFd, VEN_IOCTL_ATTACHFINISH,
                               NULL, 0, NULL, 0);
    if (rcAttachFinish < 0) {
        releaseCurrentDevice();
        closeVenHandle();
        snprintf(statusText, sizeof(statusText),
                 "not found VEN a:%d r:%d d:%d g:1 f:%d",
                 rcAttach, rcResume, rcDeviceInfo, rcAttachFinish);
        return false;
    }

    readCompleted = false;
    readInFlight = false;
    rcReadSubmit = queueInterruptIn();
    if (rcReadSubmit < 0) {
        snprintf(statusText, sizeof(statusText),
                 "not found VEN q:%d", rcReadSubmit);
        releaseCurrentDevice();
        armDeviceChange();
        return false;
    }

    static const u8 powerOn[] = { 0x05, 0x20, 0x00, 0x01, 0x00 };
    static const u8 oneSInit[] = { 0x05, 0x20, 0x00, 0x0f, 0x06 };

    rcPower = sendInterruptOut(powerOn, sizeof(powerOn), -1);
    rcSpecial = (rcPower >= 0)
              ? sendInterruptOut(oneSInit, sizeof(oneSInit), -1)
              : 999;

    if (rcPower < 0 || rcSpecial < 0) {
        snprintf(statusText, sizeof(statusText),
                 "not found VEN a:%d r:%d d:%d f:%d q:%d p:%d/%d",
                 rcAttach, rcResume, rcDeviceInfo, rcAttachFinish,
                 rcReadSubmit, rcPower, rcSpecial);
        /* Keep the owned interface and pending IN alive for diagnostics. */
        return false;
    }

    active = true;
    snprintf(statusText, sizeof(statusText),
             "connected VEN h:%d ep:%02x/%02x q:%d p:%d/%d rx:%u c:%02x",
             venHandleId, endpointIn, endpointOut, rcReadSubmit,
             rcPower, rcSpecial, (unsigned)rxPackets,
             (unsigned)lastRxCommand);
    return true;
}

static void markDisconnected(void)
{
    active = false;
    heldButtons = 0;
    guidePressed = false;
    readCompleted = false;
    readInFlight = false;
    endpointIn = endpointOut = 0;
    releaseCurrentDevice();

    if (venFd < 0 || !armDeviceChange()) {
        /* armDeviceChange returns 0 on success; leave handle open either way. */
    }
}

void XBOXONE_ScanPads(void)
{
    if (!initialized) {
        initialized = true;
        if (!allocateVenBuffers()) {
            snprintf(statusText, sizeof(statusText), "not found VEN MEM2");
            return;
        }
        initializeVenWatch();
        return;
    }

    if (venFd < 0) {
        initializeVenWatch();
        return;
    }

    if (!active) {
        if (listReady)
            connectFromReadyList();
        return;
    }

    if (readCompleted) {
        s32 result = rcReadComplete;
        readCompleted = false;

        if (result > 0) {
            u16 len = (result > MAX_REPORT_SIZE) ? MAX_REPORT_SIZE : (u16)result;
            parseXboxOneReport(inData, len);
            readRetryCountdown = 0;
        } else if (result == USB_NAK_RC) {
            readRetryCountdown = READ_RETRY_FRAMES;
        } else if (result < 0) {
            markDisconnected();
            snprintf(statusText, sizeof(statusText),
                     "not found VEN read:%d", result);
            return;
        }
    }

    if (!readInFlight) {
        if (readRetryCountdown > 0) {
            readRetryCountdown--;
        } else {
            s32 rc = queueInterruptIn();
            if (rc < 0 && rc != USB_NAK_RC) {
                markDisconnected();
                snprintf(statusText, sizeof(statusText),
                         "not found VEN queue:%d", rc);
                return;
            }
        }
    }

    snprintf(statusText, sizeof(statusText),
             "connected VEN h:%d ep:%02x/%02x q:%d p:%d/%d rr:%d rx:%u c:%02x",
             venHandleId, endpointIn, endpointOut, rcReadSubmit,
             rcPower, rcSpecial, (s32)rcReadComplete,
             (unsigned)rxPackets, (unsigned)lastRxCommand);
}

u32 XBOXONE_ButtonsHeld(int chan)
{
    if (chan != 0 || !active)
        return 0;
    return heldButtons;
}

char *XBOXONE_Status(void)
{
    return statusText;
}

#endif /* HW_RVL */