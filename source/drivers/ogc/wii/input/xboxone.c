#ifdef HW_RVL

/*
 * Wired Xbox One S input for Snes9x GX.
 *
 * Diagnostic transport matrix for 045e:02ea. One build tries, in order:
 *   v1 - private USB_VEN handle, no ATTACH/RELEASE, libogc2 lifecycle
 *   v2 - same lifecycle with IOS_Open("/dev/usb/ven", 0)
 *   v3 - libogc2 USB_GetDeviceList/OpenDevice/GetDescriptors/transfers only
 *
 * USB_VEN GetDeviceChange is always asynchronous so an unplugged controller
 * can never block the emulator's main loop. USB callbacks only update flags.
 */

#include <gccore.h>
#include <ogc/ipc.h>
#include <ogc/system.h>
#include <ogc/usb.h>
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
#define VEN_IOCTL_ATTACHFINISH     6
#define VEN_IOCTL_SUSPEND_RESUME  16
#define VEN_IOCTL_INTRMSG         19

#define MAX_REPORT_SIZE        64
#define MAX_USB_DEVICES        32
#define READ_RETRY_FRAMES       3
#define V3_POLL_FRAMES         60
#define STICK_THRESHOLD     16384
#define TRIGGER_THRESHOLD      128
#define USB_NAK_RC          (-7005)

#define VARIANT_NONE 0
#define VARIANT_V1   1
#define VARIANT_V2   2
#define VARIANT_V3   3

/* One permanent Arena2 allocation. Every IOS-visible region is 32-byte aligned. */
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
static bool terminalFailure = false;
static int attemptVariant = VARIANT_NONE;
static int selectedVariant = VARIANT_NONE;
static bool variantNeedsStart = false;

/* Direct USB_VEN state used by v1/v2. */
static s32 venFd = -1;
static int venOpenMode = -1;
static s32 deviceId = -1;
static u8 endpointIn = 0;
static u8 endpointOut = 0;
static u16 reportLength = MAX_REPORT_SIZE;
static u8 outSequence = 1;

/* libogc2 state used by v3. V5 IDs can be negative; only -1 is sentinel. */
static s32 libogcFd = -1;
static int v3PollCountdown = 0;

static volatile u32 heldButtons = 0;
static volatile bool guidePressed = false;
static volatile u32 rxPackets = 0;
static volatile u8 lastRxCommand = 0;

/* Current operation diagnostics. */
static s32 rcVersion = 999;
static volatile s32 rcList = 999;
static volatile s32 rcAttachFinish = 999;
static s32 rcResume = 999;
static s32 rcDeviceInfo = 999;
static s32 rcReadSubmit = 999;
static s32 rcPower = 999;
static s32 rcSpecial = 999;
static volatile s32 rcReadComplete = 999;
static s32 rcV3List = 999;
static s32 rcV3Open = 999;
static s32 rcV3Desc = 999;

/* Per-variant results kept for the single Credits screenshot. */
static s32 v1Resume = 999, v1Info = 999;
static s32 v2Resume = 999, v2Info = 999;
static s32 v3Open = 999, v3Desc = 999;
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

/* Direct USB_VEN event state. */
static volatile bool listInFlight = false;
static volatile bool listReady = false;
static volatile bool finishInFlight = false;
static volatile bool finishReady = false;
static s32 pendingDeviceId = -1;

/* Interrupt-IN state shared by all variants. */
static volatile bool readInFlight = false;
static volatile bool readCompleted = false;
static int readRetryCountdown = 0;

static bool allocateBuffers(void)
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

static s32 attachFinishCallback(s32 result, void *userdata)
{
    (void)userdata;
    rcAttachFinish = result;
    finishInFlight = false;
    finishReady = true;
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

static s32 queueDirectInterruptIn(void)
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

static s32 sendDirectInterruptOut(const u8 *packet, u16 len, int seqOverride)
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

static s32 queueLibogcInterruptIn(void)
{
    if (libogcFd == -1 || !endpointIn || readInFlight)
        return IPC_EINVAL;

    memset(inData, 0, MAX_REPORT_SIZE);
    s32 rc = USB_ReadIntrMsgAsync(libogcFd, endpointIn, reportLength,
                                  inData, readCallback, NULL);
    rcReadSubmit = rc;
    if (rc >= 0)
        readInFlight = true;
    return rc;
}

static s32 sendLibogcInterruptOut(const u8 *packet, u16 len, int seqOverride)
{
    if (libogcFd == -1 || !endpointOut || !packet || !len)
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
    return USB_WriteIntrMsg(libogcFd, endpointOut, len, outData);
}

static s32 queueCurrentInterruptIn(void)
{
    return (selectedVariant == VARIANT_V3) ?
        queueLibogcInterruptIn() : queueDirectInterruptIn();
}

static s32 sendCurrentInterruptOut(const u8 *packet, u16 len, int seqOverride)
{
    return (selectedVariant == VARIANT_V3) ?
        sendLibogcInterruptOut(packet, len, seqOverride) :
        sendDirectInterruptOut(packet, len, seqOverride);
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
            sendCurrentInterruptOut(ack, sizeof(ack), d[2]);
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

/* Parse raw 0xc0 USB_VEN GETDEVPARAMS response. */
static bool parseDirectGipInterface(void)
{
    const u8 *iface = deviceInfo + 52;
    if (iface[0] < 9 || iface[5] != GIP_IF_CLASS ||
        iface[6] != GIP_IF_SUBCLASS || iface[7] != GIP_IF_PROTOCOL)
        return false;

    endpointIn = endpointOut = 0;
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

/* Parse descriptors returned by libogc2. IOS58 exposes one interface per V5 ID. */
static bool parseLibogcGipInterface(const usb_devdesc *desc)
{
    endpointIn = endpointOut = 0;
    reportLength = MAX_REPORT_SIZE;
    if (!desc || !desc->configurations)
        return false;

    for (u8 c = 0; c < desc->bNumConfigurations; ++c) {
        const usb_configurationdesc *cfg = &desc->configurations[c];
        if (!cfg->interfaces)
            continue;
        for (u8 i = 0; i < cfg->bNumInterfaces; ++i) {
            const usb_interfacedesc *iface = &cfg->interfaces[i];
            if (iface->bInterfaceClass != GIP_IF_CLASS ||
                iface->bInterfaceSubClass != GIP_IF_SUBCLASS ||
                iface->bInterfaceProtocol != GIP_IF_PROTOCOL ||
                !iface->endpoints)
                continue;

            for (u8 e = 0; e < iface->bNumEndpoints; ++e) {
                const usb_endpointdesc *ep = &iface->endpoints[e];
                if ((ep->bmAttributes & 0x03) != USB_ENDPOINT_INTERRUPT)
                    continue;
                if (ep->bEndpointAddress & USB_ENDPOINT_IN) {
                    if (!endpointIn)
                        endpointIn = ep->bEndpointAddress;
                } else if (!endpointOut) {
                    endpointOut = ep->bEndpointAddress;
                }
            }
            if (endpointIn && endpointOut)
                return true;
        }
    }
    return false;
}

static void closeDirectHandle(void)
{
    /* v1/v2 intentionally never issue USB_VEN ATTACH or RELEASE. */
    if (venFd >= 0)
        IOS_Close(venFd);
    venFd = -1;
    venOpenMode = -1;
    deviceId = -1;
    listInFlight = listReady = false;
    finishInFlight = finishReady = false;
    pendingDeviceId = -1;
}

static void closeLibogcDevice(void)
{
    if (libogcFd != -1)
        USB_CloseDevice(&libogcFd);
    libogcFd = -1;
    deviceId = -1;
}

static void resetInputState(void)
{
    active = false;
    heldButtons = 0;
    guidePressed = false;
    rxPackets = 0;
    lastRxCommand = 0;
    readInFlight = false;
    readCompleted = false;
    readRetryCountdown = 0;
    endpointIn = endpointOut = 0;
    reportLength = MAX_REPORT_SIZE;
    outSequence = 1;
    selectedVariant = VARIANT_NONE;
}

static s32 openDirectHandleForVariant(int variant)
{
    if (variant == VARIANT_V2) {
        venOpenMode = 0;
        return IOS_Open(venPath, IPC_OPEN_NONE);
    }

    /* v1 preserves the separate-handle experiment, but removes ATTACH/RELEASE. */
    for (int mode = 1; mode < 16; ++mode) {
        s32 fd = IOS_Open(venPath, (u32)mode);
        if (fd >= 0) {
            venOpenMode = mode;
            return fd;
        }
    }
    return IPC_ENOENT;
}

static s32 armDeviceChange(void)
{
    if (venFd < 0 || listInFlight || finishInFlight)
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

static s32 armAttachFinish(void)
{
    if (venFd < 0 || finishInFlight)
        return IPC_EINVAL;
    finishReady = false;
    s32 rc = IOS_IoctlAsync(venFd, VEN_IOCTL_ATTACHFINISH,
                            NULL, 0, NULL, 0,
                            attachFinishCallback, NULL);
    if (rc >= 0)
        finishInFlight = true;
    else
        rcAttachFinish = rc;
    return rc;
}

static void setVariantDiagnostics(int variant, s32 resumeRc, s32 infoRc)
{
    if (variant == VARIANT_V1) {
        v1Resume = resumeRc;
        v1Info = infoRc;
    } else if (variant == VARIANT_V2) {
        v2Resume = resumeRc;
        v2Info = infoRc;
    }
}

static void requestNextVariant(void)
{
    closeDirectHandle();
    closeLibogcDevice();
    resetInputState();
    if (attemptVariant < VARIANT_V3) {
        attemptVariant++;
        variantNeedsStart = true;
    } else {
        terminalFailure = true;
        snprintf(statusText, sizeof(statusText),
                 "fail v1 r:%d d:%d | v2 r:%d d:%d | v3 o:%d d:%d",
                 v1Resume, v1Info, v2Resume, v2Info, v3Open, v3Desc);
    }
}

static bool startVariant(int variant)
{
    terminalFailure = false;
    rcVersion = rcList = rcAttachFinish = 999;
    rcResume = rcDeviceInfo = rcReadSubmit = 999;
    rcPower = rcSpecial = rcReadComplete = 999;

    if (variant == VARIANT_V3) {
        s32 rc = USB_Initialize();
        if (rc < 0) {
            v3Open = rc;
            v3Desc = 999;
            return false;
        }
        v3PollCountdown = 0;
        snprintf(statusText, sizeof(statusText),
                 "not found v3 wait | v1 %d/%d v2 %d/%d",
                 v1Resume, v1Info, v2Resume, v2Info);
        return true;
    }

    venFd = openDirectHandleForVariant(variant);
    if (venFd < 0) {
        setVariantDiagnostics(variant, venFd, 999);
        return false;
    }

    memset(versionBuffer, 0, 0x20);
    rcVersion = IOS_Ioctl(venFd, VEN_IOCTL_GETVERSION,
                          NULL, 0, versionBuffer, 0x20);
    if (rcVersion < 0 || *(u32 *)versionBuffer != 0x00050001) {
        setVariantDiagnostics(variant, rcVersion, 999);
        closeDirectHandle();
        return false;
    }

    s32 rc = armDeviceChange();
    if (rc < 0) {
        setVariantDiagnostics(variant, rc, 999);
        closeDirectHandle();
        return false;
    }

    snprintf(statusText, sizeof(statusText), "not found v%d wait m:%d",
             variant, venOpenMode);
    return true;
}

static void startPendingVariant(void)
{
    if (!variantNeedsStart || active || terminalFailure)
        return;

    variantNeedsStart = false;
    while (attemptVariant <= VARIANT_V3) {
        if (startVariant(attemptVariant))
            return;
        if (attemptVariant == VARIANT_V3) {
            terminalFailure = true;
            snprintf(statusText, sizeof(statusText),
                     "fail v1 r:%d d:%d | v2 r:%d d:%d | v3 o:%d d:%d",
                     v1Resume, v1Info, v2Resume, v2Info, v3Open, v3Desc);
            return;
        }
        attemptVariant++;
    }
}

static void finishDirectControllerInit(void)
{
    deviceId = pendingDeviceId;
    pendingDeviceId = -1;

    setCmdDevice(deviceId);
    ((s32 *)cmdBuffer)[2] = 1;
    rcResume = IOS_Ioctl(venFd, VEN_IOCTL_SUSPEND_RESUME,
                         cmdBuffer, 0x20, NULL, 0);

    setCmdDevice(deviceId);
    memset(deviceInfo, 0, 0xc0);
    rcDeviceInfo = IOS_Ioctl(venFd, VEN_IOCTL_GETDEVPARAMS,
                             cmdBuffer, 0x20, deviceInfo, 0xc0);

    setVariantDiagnostics(attemptVariant, rcResume, rcDeviceInfo);

    /* The requested matrix specifically escalates v1 -> v2 -> v3 on -4.
     * Other pre-transfer failures also advance so this one build remains useful. */
    if (rcResume < 0 || rcDeviceInfo < 0 || !parseDirectGipInterface()) {
        requestNextVariant();
        return;
    }

    selectedVariant = attemptVariant;
    readCompleted = false;
    readInFlight = false;
    rcReadSubmit = queueDirectInterruptIn();
    if (rcReadSubmit < 0) {
        requestNextVariant();
        return;
    }

    static const u8 powerOn[] = { 0x05, 0x20, 0x00, 0x01, 0x00 };
    static const u8 oneSInit[] = { 0x05, 0x20, 0x00, 0x0f, 0x06 };

    rcPower = sendDirectInterruptOut(powerOn, sizeof(powerOn), -1);
    if (rcPower < 0) {
        terminalFailure = true;
        snprintf(statusText, sizeof(statusText),
                 "not found v%d gip q:%d p:%d/%d r:%d d:%d",
                 selectedVariant, rcReadSubmit, rcPower, rcSpecial,
                 rcResume, rcDeviceInfo);
        return;
    }

    rcSpecial = sendDirectInterruptOut(oneSInit, sizeof(oneSInit), -1);
    if (rcSpecial < 0) {
        terminalFailure = true;
        snprintf(statusText, sizeof(statusText),
                 "not found v%d gip q:%d p:%d/%d r:%d d:%d",
                 selectedVariant, rcReadSubmit, rcPower, rcSpecial,
                 rcResume, rcDeviceInfo);
        return;
    }

    active = true;
    snprintf(statusText, sizeof(statusText),
             "connected v%d ep:%02x/%02x q:%d p:%d/%d rx:%u",
             selectedVariant, endpointIn, endpointOut, rcReadSubmit,
             rcPower, rcSpecial, (unsigned)rxPackets);
}

static void processDirectVariant(void)
{
    if (listReady) {
        listReady = false;
        if (rcList < 0) {
            setVariantDiagnostics(attemptVariant, rcList, 999);
            requestNextVariant();
            return;
        }

        int count = rcList;
        if (count > MAX_USB_DEVICES)
            count = MAX_USB_DEVICES;
        usb_device_entry *entries = (usb_device_entry *)deviceList;
        pendingDeviceId = -1;
        for (int i = 0; i < count; ++i) {
            if (entries[i].vid == MICROSOFT_VID &&
                entries[i].pid == XBOX_ONE_S_PID &&
                entryInterfaceNumber(&entries[i]) == 0) {
                pendingDeviceId = entries[i].device_id;
                break;
            }
        }

        /* Exactly like libogc2: every completed device-change snapshot is
         * followed by AttachFinish; no ATTACH and no RELEASE are issued. */
        s32 rc = armAttachFinish();
        if (rc < 0) {
            setVariantDiagnostics(attemptVariant, rc, 999);
            requestNextVariant();
        }
        return;
    }

    if (finishReady) {
        finishReady = false;
        if (pendingDeviceId == -1) {
            s32 rc = armDeviceChange();
            if (rc < 0) {
                setVariantDiagnostics(attemptVariant, rc, 999);
                requestNextVariant();
            } else {
                snprintf(statusText, sizeof(statusText),
                         "not found v%d wait m:%d f:%d",
                         attemptVariant, venOpenMode, (s32)rcAttachFinish);
            }
            return;
        }

        finishDirectControllerInit();
    }
}

static void processV3(void)
{
    if (v3PollCountdown > 0) {
        v3PollCountdown--;
        return;
    }
    v3PollCountdown = V3_POLL_FRAMES;

    u8 count = 0;
    memset(deviceList, 0, 0x180);
    rcV3List = USB_GetDeviceList((usb_device_entry *)deviceList,
                                 MAX_USB_DEVICES, GIP_IF_CLASS, &count);
    if (rcV3List < 0) {
        v3Open = rcV3List;
        v3Desc = 999;
        snprintf(statusText, sizeof(statusText),
                 "fail v1 %d/%d v2 %d/%d v3 list:%d",
                 v1Resume, v1Info, v2Resume, v2Info, rcV3List);
        return;
    }

    usb_device_entry *entries = (usb_device_entry *)deviceList;
    bool foundPad = false;
    rcV3Open = rcV3Desc = 999;

    for (u8 i = 0; i < count; ++i) {
        if (entries[i].vid != MICROSOFT_VID || entries[i].pid != XBOX_ONE_S_PID)
            continue;

        foundPad = true;
        s32 fd = -1;
        rcV3Open = USB_OpenDevice(entries[i].device_id,
                                  entries[i].vid, entries[i].pid, &fd);
        v3Open = rcV3Open;
        if (rcV3Open < 0 || fd == -1)
            continue;

        usb_devdesc desc;
        memset(&desc, 0, sizeof(desc));
        rcV3Desc = USB_GetDescriptors(fd, &desc);
        v3Desc = rcV3Desc;
        bool isGip = false;
        if (rcV3Desc >= 0) {
            isGip = parseLibogcGipInterface(&desc);
            USB_FreeDescriptors(&desc);
        }

        if (!isGip) {
            USB_CloseDevice(&fd);
            continue;
        }

        libogcFd = fd; /* may be negative; -1 alone is invalid */
        deviceId = entries[i].device_id;
        selectedVariant = VARIANT_V3;
        outSequence = 1;
        readCompleted = false;
        readInFlight = false;

        rcReadSubmit = queueLibogcInterruptIn();
        if (rcReadSubmit < 0) {
            closeLibogcDevice();
            selectedVariant = VARIANT_NONE;
            continue;
        }

        static const u8 powerOn[] = { 0x05, 0x20, 0x00, 0x01, 0x00 };
        static const u8 oneSInit[] = { 0x05, 0x20, 0x00, 0x0f, 0x06 };
        rcPower = sendLibogcInterruptOut(powerOn, sizeof(powerOn), -1);
        if (rcPower < 0) {
            terminalFailure = true;
            snprintf(statusText, sizeof(statusText),
                     "fail v1 %d/%d v2 %d/%d v3 o:%d d:%d p:%d",
                     v1Resume, v1Info, v2Resume, v2Info,
                     v3Open, v3Desc, rcPower);
            return;
        }
        rcSpecial = sendLibogcInterruptOut(oneSInit, sizeof(oneSInit), -1);
        if (rcSpecial < 0) {
            terminalFailure = true;
            snprintf(statusText, sizeof(statusText),
                     "fail v1 %d/%d v2 %d/%d v3 o:%d d:%d p:%d/%d",
                     v1Resume, v1Info, v2Resume, v2Info,
                     v3Open, v3Desc, rcPower, rcSpecial);
            return;
        }

        active = true;
        snprintf(statusText, sizeof(statusText),
                 "connected v3 ep:%02x/%02x q:%d p:%d/%d rx:%u",
                 endpointIn, endpointOut, rcReadSubmit,
                 rcPower, rcSpecial, (unsigned)rxPackets);
        return;
    }

    if (!foundPad) {
        snprintf(statusText, sizeof(statusText),
                 "not found v3 wait | v1 %d/%d v2 %d/%d",
                 v1Resume, v1Info, v2Resume, v2Info);
    } else {
        snprintf(statusText, sizeof(statusText),
                 "fail v1 %d/%d v2 %d/%d v3 o:%d d:%d",
                 v1Resume, v1Info, v2Resume, v2Info, v3Open, v3Desc);
    }
}

static void handleActiveRead(void)
{
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
            int previous = selectedVariant;
            if (previous == VARIANT_V3)
                closeLibogcDevice();
            else
                closeDirectHandle();
            resetInputState();
            attemptVariant = VARIANT_V1;
            variantNeedsStart = true;
            snprintf(statusText, sizeof(statusText),
                     "not found v%d read:%d", previous, result);
            return;
        }
    }

    if (!readInFlight) {
        if (readRetryCountdown > 0) {
            readRetryCountdown--;
        } else {
            s32 rc = queueCurrentInterruptIn();
            if (rc < 0 && rc != USB_NAK_RC) {
                int previous = selectedVariant;
                if (previous == VARIANT_V3)
                    closeLibogcDevice();
                else
                    closeDirectHandle();
                resetInputState();
                attemptVariant = VARIANT_V1;
                variantNeedsStart = true;
                snprintf(statusText, sizeof(statusText),
                         "not found v%d queue:%d", previous, rc);
                return;
            }
        }
    }

    snprintf(statusText, sizeof(statusText),
             "connected v%d ep:%02x/%02x q:%d p:%d/%d rr:%d rx:%u c:%02x",
             selectedVariant, endpointIn, endpointOut, rcReadSubmit,
             rcPower, rcSpecial, (s32)rcReadComplete,
             (unsigned)rxPackets, (unsigned)lastRxCommand);
}

void XBOXONE_ScanPads(void)
{
    if (!initialized) {
        initialized = true;
        if (!allocateBuffers()) {
            snprintf(statusText, sizeof(statusText), "not found USB MEM2");
            terminalFailure = true;
            return;
        }
        attemptVariant = VARIANT_V1;
        variantNeedsStart = true;
    }

    if (active) {
        handleActiveRead();
        return;
    }

    if (terminalFailure)
        return;

    startPendingVariant();
    if (variantNeedsStart || terminalFailure)
        return;

    if (attemptVariant == VARIANT_V1 || attemptVariant == VARIANT_V2)
        processDirectVariant();
    else if (attemptVariant == VARIANT_V3)
        processV3();
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
