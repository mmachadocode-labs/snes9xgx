#include <gccore.h>
#include <ogc/ipc.h>
#include <ogc/system.h>
#include <ogc/usb.h>
#include <wiiuse/wpad.h>

#include <stdio.h>
#include <string.h>

#define MICROSOFT_VID  0x045e
#define XBOX_ONE_S_PID 0x02ea

#define VEN_IOCTL_GETVERSION       0
#define VEN_IOCTL_GETDEVICECHANGE  1
#define VEN_IOCTL_GETDEVPARAMS     3
#define VEN_IOCTL_ATTACH           4
#define VEN_IOCTL_RELEASE          5
#define VEN_IOCTL_ATTACHFINISH     6
#define VEN_IOCTL_SUSPEND_RESUME  16
#define VEN_IOCTL_INTRMSG         19

#define GIP_CLASS    0xff
#define GIP_SUBCLASS 0x47
#define GIP_PROTOCOL 0xd0

#define MAX_USB_DEVICES 32
#define MAX_REPORT_SIZE 64
#define USB_NAK_RC (-7005)

#define BLOCK_SIZE      0x700
#define OFF_LIST        0x000
#define OFF_CMD         0x180
#define OFF_INFO        0x1a0
#define OFF_VERSION     0x260
#define OFF_IN_HEADER   0x2a0
#define OFF_IN_DATA     0x2e0
#define OFF_OUT_HEADER  0x320
#define OFF_OUT_DATA    0x360
#define OFF_IN_VEC      0x3a0
#define OFF_OUT_VEC     0x3e0

static const char venPath[] ATTRIBUTE_ALIGN(32) = "/dev/usb/ven";

static u8 *block;
static u8 *deviceList;
static u8 *cmdBuffer;
static u8 *deviceInfo;
static u8 *versionBuffer;
static u8 *inHeader;
static u8 *inData;
static u8 *outHeader;
static u8 *outData;
static ioctlv *inVec;
static ioctlv *outVec;

static s32 venFd = -1;
static s32 deviceId = -1;
static u8 endpointIn = 0;
static u8 endpointOut = 0;
static u16 reportLength = MAX_REPORT_SIZE;
static u8 outSequence = 1;

static volatile bool listReady = false;
static volatile bool listInFlight = false;
static volatile s32 rcList = 999;

static volatile bool readReady = false;
static volatile bool readInFlight = false;
static volatile s32 rcReadComplete = 999;

static s32 rcOpen = 999;
static s32 rcVersion = 999;
static s32 rcAttach = 999;
static s32 rcAttachFinish = 999;
static s32 rcResume = 999;
static s32 rcDeviceInfo = 999;
static s32 rcReadSubmit = 999;
static s32 rcPower = 999;
static s32 rcSpecial = 999;
static s32 rcRelease = 999;

static int matchingEntries = 0;
static s32 foundIds[4] = { -1, -1, -1, -1 };
static u8 foundIfs[4] = { 0 };
static u32 rxPackets = 0;
static u8 lastPacket[32];
static u32 lastPacketLen = 0;
static bool testRan = false;
static bool deviceOwned = false;

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
    readReady = true;
    return 0;
}

static bool allocateBuffers(void)
{
    block = (u8 *)SYS_AllocArenaMem2Lo(BLOCK_SIZE, 32);
    if (!block)
        return false;

    memset(block, 0, BLOCK_SIZE);
    deviceList   = block + OFF_LIST;
    cmdBuffer    = block + OFF_CMD;
    deviceInfo   = block + OFF_INFO;
    versionBuffer= block + OFF_VERSION;
    inHeader     = block + OFF_IN_HEADER;
    inData       = block + OFF_IN_DATA;
    outHeader    = block + OFF_OUT_HEADER;
    outData      = block + OFF_OUT_DATA;
    inVec        = (ioctlv *)(block + OFF_IN_VEC);
    outVec       = (ioctlv *)(block + OFF_OUT_VEC);
    return true;
}

static void setCmdDevice(s32 id)
{
    memset(cmdBuffer, 0, 0x20);
    ((s32 *)cmdBuffer)[0] = id;
}

static u8 interfaceNumber(const usb_device_entry *entry)
{
    return (u8)((entry->token >> 8) & 0xff);
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

static bool parseGipInterface(void)
{
    const u8 *iface = deviceInfo + 52;

    endpointIn = 0;
    endpointOut = 0;
    reportLength = MAX_REPORT_SIZE;

    if (iface[0] < 9)
        return false;

    if (iface[5] != GIP_CLASS || iface[6] != GIP_SUBCLASS || iface[7] != GIP_PROTOCOL)
        return false;

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

static void buildTransferHeader(u8 *header, u8 *data, u16 len, u8 endpoint)
{
    memset(header, 0, 64);
    ((s32 *)header)[0] = deviceId;
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

static s32 sendInterruptOut(const u8 *packet, u16 len)
{
    if (venFd < 0 || deviceId == -1 || !endpointOut)
        return IPC_EINVAL;

    memset(outData, 0, MAX_REPORT_SIZE);
    memcpy(outData, packet, len);
    outData[2] = outSequence++;
    if (outSequence == 0)
        outSequence = 1;

    buildTransferHeader(outHeader, outData, len, endpointOut);
    outVec[0].data = outHeader;
    outVec[0].len = 64;
    outVec[1].data = outData;
    outVec[1].len = len;

    return IOS_Ioctlv(venFd, VEN_IOCTL_INTRMSG, 2, 0, outVec);
}

static void releaseDevice(void)
{
    if (!deviceOwned || venFd < 0 || deviceId == -1)
        return;

    setCmdDevice(deviceId);
    rcRelease = IOS_Ioctl(venFd, VEN_IOCTL_RELEASE, cmdBuffer, 0x20, NULL, 0);
    deviceOwned = false;
}

static void runControllerTest(void)
{
    testRan = true;

    setCmdDevice(deviceId);
    rcAttach = IOS_Ioctl(venFd, VEN_IOCTL_ATTACH, cmdBuffer, 0x20, NULL, 0);
    if (rcAttach >= 0)
        deviceOwned = true;

    rcAttachFinish = IOS_Ioctl(venFd, VEN_IOCTL_ATTACHFINISH, NULL, 0, NULL, 0);

    if (rcAttach < 0)
        return;

    setCmdDevice(deviceId);
    ((s32 *)cmdBuffer)[2] = 1;
    rcResume = IOS_Ioctl(venFd, VEN_IOCTL_SUSPEND_RESUME, cmdBuffer, 0x20, NULL, 0);

    setCmdDevice(deviceId);
    memset(deviceInfo, 0, 0xc0);
    rcDeviceInfo = IOS_Ioctl(venFd, VEN_IOCTL_GETDEVPARAMS,
                             cmdBuffer, 0x20, deviceInfo, 0xc0);

    if (rcDeviceInfo < 0 || !parseGipInterface())
        return;

    readReady = false;
    readInFlight = false;
    rcReadSubmit = queueInterruptIn();

    static const u8 powerOn[]  = { 0x05, 0x20, 0x00, 0x01, 0x00 };
    static const u8 oneSInit[] = { 0x05, 0x20, 0x00, 0x0f, 0x06 };

    rcPower = sendInterruptOut(powerOn, sizeof(powerOn));
    rcSpecial = sendInterruptOut(oneSInit, sizeof(oneSInit));
}

static void processDeviceList(void)
{
    listReady = false;

    matchingEntries = 0;
    for (int i = 0; i < 4; ++i) {
        foundIds[i] = -1;
        foundIfs[i] = 0;
    }

    if (rcList < 0)
        return;

    int count = rcList;
    if (count > MAX_USB_DEVICES)
        count = MAX_USB_DEVICES;

    usb_device_entry *entries = (usb_device_entry *)deviceList;
    deviceId = -1;

    for (int i = 0; i < count; ++i) {
        if (entries[i].vid != MICROSOFT_VID || entries[i].pid != XBOX_ONE_S_PID)
            continue;

        u8 ifnum = interfaceNumber(&entries[i]);
        if (matchingEntries < 4) {
            foundIds[matchingEntries] = entries[i].device_id;
            foundIfs[matchingEntries] = ifnum;
        }
        matchingEntries++;

        if (ifnum == 0 && deviceId == -1)
            deviceId = entries[i].device_id;
    }

    if (deviceId != -1) {
        runControllerTest();
        return;
    }

    rcAttachFinish = IOS_Ioctl(venFd, VEN_IOCTL_ATTACHFINISH, NULL, 0, NULL, 0);
    armDeviceChange();
}

static void processRead(void)
{
    if (!readReady)
        return;

    readReady = false;
    s32 result = rcReadComplete;

    if (result > 0) {
        rxPackets++;
        lastPacketLen = (result > (s32)sizeof(lastPacket)) ? sizeof(lastPacket) : (u32)result;
        memcpy(lastPacket, inData, lastPacketLen);
    }

    if (result >= 0 || result == USB_NAK_RC)
        queueInterruptIn();
}

static void drawScreen(void)
{
    printf("\x1b[2J\x1b[H");
    printf("Xbox One S USB standalone test - Wii / IOS58\n");
    printf("Target: 045e:02ea   HOME: exit\n\n");

    printf("/dev/usb/ven mode 0\n");
    printf("open:%d  version:%d  list:%d\n", rcOpen, rcVersion, (s32)rcList);
    printf("matching interfaces: %d\n", matchingEntries);

    for (int i = 0; i < matchingEntries && i < 4; ++i)
        printf("  #%d id:%d if:%u\n", i, foundIds[i], foundIfs[i]);

    if (!testRan) {
        printf("\nWaiting for Xbox One S...\n");
    } else {
        printf("\nOwnership / descriptors\n");
        printf("attach:%d finish:%d resume:%d getparams:%d\n",
               rcAttach, rcAttachFinish, rcResume, rcDeviceInfo);
        printf("GIP endpoints: IN=%02x OUT=%02x len=%u\n",
               endpointIn, endpointOut, reportLength);

        printf("\nGIP transport\n");
        printf("read submit:%d  power:%d  init:%d  read complete:%d\n",
               rcReadSubmit, rcPower, rcSpecial, (s32)rcReadComplete);
        printf("rx packets:%u\n", (unsigned)rxPackets);

        if (lastPacketLen) {
            printf("last packet (%u): ", (unsigned)lastPacketLen);
            for (u32 i = 0; i < lastPacketLen; ++i) {
                printf("%02x ", lastPacket[i]);
                if ((i + 1) % 16 == 0 && i + 1 < lastPacketLen)
                    printf("\n                  ");
            }
            printf("\n");
        }

        if (rcAttach < 0)
            printf("\nSTOP: ownership failed at ATTACH.\n");
        else if (rcResume < 0)
            printf("\nSTOP: ATTACH worked, RESUME failed.\n");
        else if (rcDeviceInfo < 0)
            printf("\nSTOP: RESUME worked, GETDEVPARAMS failed.\n");
        else if (!endpointIn || !endpointOut)
            printf("\nSTOP: descriptors read, FF/47/D0 endpoints not found.\n");
        else if (rcPower < 0 || rcSpecial < 0)
            printf("\nSTOP: descriptors/endpoints worked, GIP OUT failed.\n");
        else if (rxPackets == 0)
            printf("\nGIP OUT accepted. Waiting for IN packets...\n");
        else
            printf("\nSUCCESS: Xbox packets are arriving.\n");
    }

    if (deviceOwned)
        printf("\nDevice owned by test handle. release:%d\n", rcRelease);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    VIDEO_Init();
    WPAD_Init();

    GXRModeObj *rmode = VIDEO_GetPreferredMode(NULL);
    void *xfb = MEM_K0_TO_K1(SYS_AllocateFramebuffer(rmode));
    console_init(xfb, 20, 20, rmode->fbWidth, rmode->xfbHeight,
                 rmode->fbWidth * VI_DISPLAY_PIX_SZ);

    VIDEO_Configure(rmode);
    VIDEO_SetNextFramebuffer(xfb);
    VIDEO_SetBlack(FALSE);
    VIDEO_Flush();
    VIDEO_WaitVSync();
    if (rmode->viTVMode & VI_NON_INTERLACE)
        VIDEO_WaitVSync();

    if (!allocateBuffers()) {
        printf("Failed to allocate MEM2 buffers.\n");
        while (1)
            VIDEO_WaitVSync();
    }

    venFd = IOS_Open(venPath, IPC_OPEN_NONE);
    rcOpen = venFd;

    if (venFd >= 0) {
        memset(versionBuffer, 0, 0x20);
        rcVersion = IOS_Ioctl(venFd, VEN_IOCTL_GETVERSION,
                              NULL, 0, versionBuffer, 0x20);
        armDeviceChange();
    }

    int redraw = 0;

    while (1) {
        WPAD_ScanPads();
        if (WPAD_ButtonsDown(0) & WPAD_BUTTON_HOME)
            break;

        if (listReady && !testRan)
            processDeviceList();

        processRead();

        if (++redraw >= 15) {
            redraw = 0;
            drawScreen();
        }

        VIDEO_WaitVSync();
    }

    releaseDevice();
    if (venFd >= 0)
        IOS_Close(venFd);

    return 0;
}
