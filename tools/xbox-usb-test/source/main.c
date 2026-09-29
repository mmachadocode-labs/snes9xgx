#include <gccore.h>
#include <ogc/usb.h>
#include <wiiuse/wpad.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define TARGET_VID 0x045e
#define TARGET_PID 0x02ea
#define TARGET_IF  0
#define GIP_CLASS  0xff
#define GIP_SUBCLASS 0x47
#define GIP_PROTOCOL 0xd0
#define DEFAULT_EP_IN  0x82
#define DEFAULT_EP_OUT 0x02
#define PKT_SIZE   64
#define MAX_DEVS   32
#define USB_NAK_RC (-7005)
#define ATTACH_WAIT_FRAMES 120

#define WRITE_NONE    0
#define WRITE_POWER   1
#define WRITE_SPECIAL 2
#define WRITE_ACK     3

static usb_device_entry devices[MAX_DEVS] ATTRIBUTE_ALIGN(32);
static u8 inBuf[PKT_SIZE] ATTRIBUTE_ALIGN(32);
static u8 outBuf[PKT_SIZE] ATTRIBUTE_ALIGN(32);
static u8 lastPacket[PKT_SIZE] ATTRIBUTE_ALIGN(32);
static u8 configBuf[32] ATTRIBUTE_ALIGN(32);

static s32 usbInitRc = 999;
static s32 listRc = 999;
static u8 deviceCount = 0;
static s32 targetDeviceId = -1;
static u32 targetToken = 0;
static u32 firstSeenFrame = 0;
static bool opened = false;
static s32 usbFd = -1;
static s32 openRc = 999;
static s32 secondResumeRc = 999;
static s32 descRc = 999;
static s32 getConfigRc = 999;
static s32 setConfigRc = 999;
static s32 setAltRc = 999;
static s32 removeHookRc = 999;
static u8 currentConfig = 0xff;
static u8 desiredConfig = 1;
static u8 actualIf = 0xff;
static u8 actualAlt = 0xff;
static u8 actualClass = 0xff;
static u8 actualSub = 0xff;
static u8 actualProto = 0xff;
static u8 epIn = DEFAULT_EP_IN;
static u8 epOut = DEFAULT_EP_OUT;
static u16 epInMax = 0;
static u16 epOutMax = 0;
static bool descriptorsUseful = false;
static volatile bool removed = false;

static volatile bool readInFlight = false;
static volatile bool readDone = false;
static volatile s32 readResult = 999;
static s32 readSubmitRc = 999;
static s32 lastReadRc = 999;
static u32 readNakCount = 0;
static u32 rxPackets = 0;
static u32 inputPackets = 0;
static u8 lastRawLen = 0;
static bool readingEnabled = false;
static int readRetryDelay = 0;

static volatile bool writeInFlight = false;
static volatile bool writeDone = false;
static volatile s32 writeResult = 999;
static int currentWriteKind = WRITE_NONE;
static int pendingWriteKind = WRITE_NONE;
static int writeAttempt = 0;
static int writeDelay = 0;
static u8 outSeq = 1;
static s32 powerSubmitRc = 999;
static s32 powerDoneRc = 999;
static s32 specialSubmitRc = 999;
static s32 specialDoneRc = 999;
static s32 ackSubmitRc = 999;
static s32 ackDoneRc = 999;
static bool pendingAck = false;
static u8 pendingAckSeq = 0;

static u32 frameCounter = 0;
static u8 btn0 = 0;
static u8 btn1 = 0;
static u16 lt = 0;
static u16 rt = 0;
static s16 lx = 0;
static s16 ly = 0;
static s16 rx = 0;
static s16 ry = 0;
static bool guide = false;
static u8 lastCmd = 0;
static u8 lastSeq = 0;

static const u8 powerOn[] = { 0x05, 0x20, 0x00, 0x01, 0x00 };
static const u8 oneSInit[] = { 0x05, 0x20, 0x00, 0x0f, 0x06 };
static const u8 guideAck[] = {
    0x01, 0x20, 0x00, 0x09, 0x00, 0x07, 0x20,
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00
};

static u8 tokenInterface(const usb_device_entry *entry)
{
    return (u8)((entry->token >> 8) & 0xff);
}

static u16 readU16LE(const u8 *p)
{
    return (u16)p[0] | ((u16)p[1] << 8);
}

static s16 readS16LE(const u8 *p)
{
    return (s16)readU16LE(p);
}

static s32 readCallback(s32 result, void *userdata)
{
    (void)userdata;
    readResult = result;
    readInFlight = false;
    readDone = true;
    return 0;
}

static s32 writeCallback(s32 result, void *userdata)
{
    (void)userdata;
    writeResult = result;
    writeInFlight = false;
    writeDone = true;
    return 0;
}

static s32 removalCallback(s32 result, void *userdata)
{
    (void)result;
    (void)userdata;
    removed = true;
    return 0;
}

static void resetInputState(void)
{
    btn0 = btn1 = 0;
    lt = rt = 0;
    lx = ly = rx = ry = 0;
    guide = false;
    lastCmd = lastSeq = 0;
    memset(lastPacket, 0, sizeof(lastPacket));
    lastRawLen = 0;
}

static void resetTransportState(void)
{
    readInFlight = false;
    readDone = false;
    readResult = 999;
    readSubmitRc = 999;
    lastReadRc = 999;
    readNakCount = 0;
    rxPackets = 0;
    inputPackets = 0;
    readingEnabled = false;
    readRetryDelay = 0;

    writeInFlight = false;
    writeDone = false;
    writeResult = 999;
    currentWriteKind = WRITE_NONE;
    pendingWriteKind = WRITE_NONE;
    writeAttempt = 0;
    writeDelay = 0;
    outSeq = 1;
    powerSubmitRc = powerDoneRc = 999;
    specialSubmitRc = specialDoneRc = 999;
    ackSubmitRc = ackDoneRc = 999;
    pendingAck = false;
    pendingAckSeq = 0;
    resetInputState();
}

static void resetOpenState(void)
{
    targetDeviceId = -1;
    targetToken = 0;
    firstSeenFrame = 0;
    opened = false;
    usbFd = -1;
    openRc = secondResumeRc = descRc = 999;
    getConfigRc = setConfigRc = setAltRc = removeHookRc = 999;
    currentConfig = 0xff;
    desiredConfig = 1;
    actualIf = actualAlt = actualClass = actualSub = actualProto = 0xff;
    epIn = DEFAULT_EP_IN;
    epOut = DEFAULT_EP_OUT;
    epInMax = epOutMax = 0;
    descriptorsUseful = false;
}

static void disconnectController(void)
{
    if (usbFd != -1) {
        s32 closeFd = usbFd;
        USB_CloseDevice(&closeFd);
    }
    removed = false;
    resetTransportState();
    resetOpenState();
}

static void inspectDescriptors(void)
{
    usb_devdesc udd;
    memset(&udd, 0, sizeof(udd));
    descRc = USB_GetDescriptors(usbFd, &udd);
    if (descRc < 0)
        return;

    for (u32 c = 0; c < udd.bNumConfigurations; ++c) {
        usb_configurationdesc *ucd = &udd.configurations[c];
        for (u32 i = 0; i < ucd->bNumInterfaces; ++i) {
            usb_interfacedesc *uid = &ucd->interfaces[i];
            if (uid->bInterfaceNumber != TARGET_IF)
                continue;

            actualIf = uid->bInterfaceNumber;
            actualAlt = uid->bAlternateSetting;
            actualClass = uid->bInterfaceClass;
            actualSub = uid->bInterfaceSubClass;
            actualProto = uid->bInterfaceProtocol;
            desiredConfig = ucd->bConfigurationValue;

            u8 foundIn = 0, foundOut = 0;
            u16 foundInMax = 0, foundOutMax = 0;
            for (u32 e = 0; e < uid->bNumEndpoints; ++e) {
                usb_endpointdesc *ep = &uid->endpoints[e];
                if ((ep->bmAttributes & 0x03) != USB_ENDPOINT_INTERRUPT)
                    continue;
                if (ep->bEndpointAddress & USB_ENDPOINT_IN) {
                    foundIn = ep->bEndpointAddress;
                    foundInMax = ep->wMaxPacketSize;
                } else {
                    foundOut = ep->bEndpointAddress;
                    foundOutMax = ep->wMaxPacketSize;
                }
            }

            if (foundIn && foundOut) {
                epIn = foundIn;
                epOut = foundOut;
                epInMax = foundInMax;
                epOutMax = foundOutMax;
                descriptorsUseful = true;
            }
        }
    }

    USB_FreeDescriptors(&udd);
}

static s32 submitWrite(int kind, const u8 *packet, u16 len, int seqOverride)
{
    if (!opened || usbFd == -1 || writeInFlight || !packet || len == 0 || len > PKT_SIZE)
        return -4;

    memset(outBuf, 0, sizeof(outBuf));
    memcpy(outBuf, packet, len);
    if (seqOverride >= 0) {
        outBuf[2] = (u8)seqOverride;
    } else {
        outBuf[2] = outSeq++;
        if (outSeq == 0)
            outSeq = 1;
    }

    currentWriteKind = kind;
    writeDone = false;
    writeResult = 999;
    writeInFlight = true;
    s32 rc = USB_WriteIntrMsgAsync(usbFd, epOut, len, outBuf, writeCallback, NULL);
    if (rc < 0) {
        writeInFlight = false;
        writeResult = rc;
        writeDone = true;
    }
    return rc;
}

static void scheduleWrite(int kind, int delayFrames, bool resetAttempts)
{
    pendingWriteKind = kind;
    writeDelay = delayFrames;
    if (resetAttempts)
        writeAttempt = 0;
}

static void startScheduledWrite(void)
{
    if (pendingWriteKind == WRITE_NONE || writeInFlight || writeDone)
        return;
    if (writeDelay > 0) {
        writeDelay--;
        return;
    }

    int kind = pendingWriteKind;
    pendingWriteKind = WRITE_NONE;
    writeAttempt++;
    s32 rc;
    if (kind == WRITE_POWER) {
        rc = submitWrite(kind, powerOn, sizeof(powerOn), -1);
        powerSubmitRc = rc;
    } else if (kind == WRITE_SPECIAL) {
        rc = submitWrite(kind, oneSInit, sizeof(oneSInit), -1);
        specialSubmitRc = rc;
    } else {
        rc = submitWrite(kind, guideAck, sizeof(guideAck), pendingAckSeq);
        ackSubmitRc = rc;
    }
}

static s32 queueRead(void)
{
    if (!opened || usbFd == -1 || readInFlight)
        return -4;
    memset(inBuf, 0, sizeof(inBuf));
    readDone = false;
    readResult = 999;
    readInFlight = true;
    s32 rc = USB_ReadIntrMsgAsync(usbFd, epIn, PKT_SIZE, inBuf, readCallback, NULL);
    readSubmitRc = rc;
    if (rc < 0) {
        readInFlight = false;
        readResult = rc;
        readDone = true;
    }
    return rc;
}

static void parsePacket(const u8 *d, u8 len)
{
    if (!d || len < 1)
        return;
    lastCmd = d[0];
    if (len > 2)
        lastSeq = d[2];

    if (d[0] == 0x20 && len >= 18) {
        btn0 = d[4];
        btn1 = d[5];
        lt = readU16LE(&d[6]);
        rt = readU16LE(&d[8]);
        lx = readS16LE(&d[10]);
        ly = readS16LE(&d[12]);
        rx = readS16LE(&d[14]);
        ry = readS16LE(&d[16]);
        inputPackets++;
    } else if (d[0] == 0x07 && len >= 5) {
        guide = (d[4] & 0x01) != 0;
        if (d[1] & 0x10) {
            pendingAck = true;
            pendingAckSeq = d[2];
        }
    }
}

static void processRead(void)
{
    if (readDone) {
        s32 result = readResult;
        readDone = false;
        lastReadRc = result;
        if (result == USB_NAK_RC) {
            readNakCount++;
            readRetryDelay = 2;
        } else if (result >= 0) {
            u8 len = (result > 0 && result <= PKT_SIZE) ? (u8)result : PKT_SIZE;
            memcpy(lastPacket, inBuf, len);
            if (len < PKT_SIZE)
                memset(lastPacket + len, 0, PKT_SIZE - len);
            lastRawLen = len;
            rxPackets++;
            parsePacket(lastPacket, len);
            readRetryDelay = 0;
        } else {
            readRetryDelay = 15;
        }
    }

    if (!readingEnabled || readInFlight)
        return;
    if (readRetryDelay > 0) {
        readRetryDelay--;
        return;
    }
    queueRead();
}

static void processWrite(void)
{
    if (writeDone) {
        s32 result = writeResult;
        int kind = currentWriteKind;
        writeDone = false;
        currentWriteKind = WRITE_NONE;

        if (kind == WRITE_POWER) {
            powerDoneRc = result;
            if (result < 0 && writeAttempt < 3)
                scheduleWrite(WRITE_POWER, 12, false);
            else
                scheduleWrite(WRITE_SPECIAL, 8, true);
        } else if (kind == WRITE_SPECIAL) {
            specialDoneRc = result;
            if (result < 0 && writeAttempt < 3)
                scheduleWrite(WRITE_SPECIAL, 12, false);
            else {
                pendingWriteKind = WRITE_NONE;
                writeAttempt = 0;
            }
        } else if (kind == WRITE_ACK) {
            ackDoneRc = result;
            pendingAck = false;
            writeAttempt = 0;
        }
    }

    if (pendingAck && pendingWriteKind == WRITE_NONE && !writeInFlight && !writeDone)
        scheduleWrite(WRITE_ACK, 0, true);
    startScheduledWrite();
}

static void refreshDeviceList(void)
{
    memset(devices, 0, sizeof(devices));
    deviceCount = 0;
    listRc = USB_GetDeviceList(devices, MAX_DEVS, GIP_CLASS, &deviceCount);
    if (listRc < 0)
        return;

    s32 newTarget = -1;
    u32 newToken = 0;
    for (u8 i = 0; i < deviceCount; ++i) {
        if (devices[i].vid == TARGET_VID && devices[i].pid == TARGET_PID &&
            tokenInterface(&devices[i]) == TARGET_IF) {
            newTarget = devices[i].device_id;
            newToken = devices[i].token;
            break;
        }
    }

    if (!opened) {
        if (targetDeviceId != newTarget) {
            targetDeviceId = newTarget;
            targetToken = newToken;
            firstSeenFrame = frameCounter;
        }
    }
}

static void openAndConfigure(void)
{
    if (opened || targetDeviceId == -1)
        return;
    if ((frameCounter - firstSeenFrame) < ATTACH_WAIT_FRAMES)
        return;

    s32 candidateFd = -1;
    openRc = USB_OpenDevice(targetDeviceId, TARGET_VID, TARGET_PID, &candidateFd);
    if (openRc < 0)
        return;

    usbFd = candidateFd;
    opened = true;
    resetTransportState();

    /* USB_OpenDevice already issues USB_ResumeDevice for V5 devices. A second
       Resume often returns -4 even on working devices, so record it only as a
       diagnostic and do not use it as the success criterion. */
    secondResumeRc = USB_ResumeDevice(usbFd);

    inspectDescriptors();

    memset(configBuf, 0, sizeof(configBuf));
    getConfigRc = USB_GetConfiguration(usbFd, configBuf);
    if (getConfigRc >= 0)
        currentConfig = configBuf[0];

    if (getConfigRc < 0 || currentConfig != desiredConfig)
        setConfigRc = USB_SetConfiguration(usbFd, desiredConfig);
    else
        setConfigRc = 0;

    /* Explicitly select the interface/alt setting before touching endpoints.
       If descriptors were unavailable, use the normal Xbox data interface 0,
       alt setting 0. */
    u8 ifToSet = (actualIf == 0xff) ? TARGET_IF : actualIf;
    u8 altToSet = (actualAlt == 0xff) ? 0 : actualAlt;
    setAltRc = USB_SetAlternativeInterface(usbFd, ifToSet, altToSet);

    removeHookRc = USB_DeviceRemovalNotifyAsync(usbFd, removalCallback, NULL);

    /* Keep an IN request alive while the GIP wake sequence is being sent. */
    readingEnabled = true;
    readRetryDelay = 1;
    scheduleWrite(WRITE_POWER, 4, true);
}

static void appendButton(char *dst, size_t size, const char *name)
{
    size_t used = strlen(dst);
    if (used && used + 1 < size) {
        dst[used++] = ' ';
        dst[used] = 0;
    }
    if (used < size - 1)
        strncat(dst, name, size - used - 1);
}

static void buildButtonString(char *dst, size_t size)
{
    dst[0] = 0;
    if (btn0 & 0x10) appendButton(dst, size, "A");
    if (btn0 & 0x20) appendButton(dst, size, "B");
    if (btn0 & 0x40) appendButton(dst, size, "X");
    if (btn0 & 0x80) appendButton(dst, size, "Y");
    if (btn0 & 0x04) appendButton(dst, size, "MENU");
    if (btn0 & 0x08) appendButton(dst, size, "VIEW");
    if (btn1 & 0x01) appendButton(dst, size, "UP");
    if (btn1 & 0x02) appendButton(dst, size, "DOWN");
    if (btn1 & 0x04) appendButton(dst, size, "LEFT");
    if (btn1 & 0x08) appendButton(dst, size, "RIGHT");
    if (btn1 & 0x10) appendButton(dst, size, "LB");
    if (btn1 & 0x20) appendButton(dst, size, "RB");
    if (btn1 & 0x40) appendButton(dst, size, "LS");
    if (btn1 & 0x80) appendButton(dst, size, "RS");
    if (lt > 30) appendButton(dst, size, "LT");
    if (rt > 30) appendButton(dst, size, "RT");
    if (guide) appendButton(dst, size, "GUIDE");
    if (!dst[0])
        strncpy(dst, "(none)", size - 1);
    dst[size - 1] = 0;
}

static const char *stageName(void)
{
    if (targetDeviceId == -1)
        return "ENUMERATING";
    if (!opened)
        return "WAITING FOR LIBOGC DEVICE LIST";
    if (rxPackets == 0)
        return "CONFIGURED - WAITING FOR USB RX";
    if (inputPackets == 0)
        return "USB RX - WAITING FOR GIP INPUT";
    return "INPUT LIVE";
}

static void render(void)
{
    char buttons[160];
    buildButtonString(buttons, sizeof(buttons));

    printf("\x1b[2J\x1b[H");
    printf("XBOX ONE S USB INPUT TEST v1.2 - Wii / libogc2\n");
    printf("Target 045e:02ea  data IF:0  GIP ff/47/d0\n");
    printf("HOME: exit\n\n");
    printf("Stage: %s\n", stageName());
    printf("USB_Init:%ld GetDeviceList:%ld count:%u\n",
           (long)usbInitRc, (long)listRc, (unsigned)deviceCount);

    for (u8 i = 0; i < deviceCount && i < 6; ++i) {
        if (devices[i].vid == TARGET_VID && devices[i].pid == TARGET_PID) {
            printf(" %c dev:%ld token:%08lx token-if:%u\n",
                   devices[i].device_id == targetDeviceId ? '>' : ' ',
                   (long)devices[i].device_id, (unsigned long)devices[i].token,
                   (unsigned)tokenInterface(&devices[i]));
        }
    }

    printf("\nOpen:%ld fd:%ld  second Resume:%ld  removal:%ld\n",
           (long)openRc, (long)usbFd, (long)secondResumeRc, (long)removeHookRc);
    printf("Descriptors:%ld useful:%d  IF:%02x ALT:%02x cls:%02x/%02x/%02x\n",
           (long)descRc, descriptorsUseful ? 1 : 0,
           actualIf, actualAlt, actualClass, actualSub, actualProto);
    printf("Config get:%ld value:%02x desired:%02x set:%ld  set-alt:%ld\n",
           (long)getConfigRc, currentConfig, desiredConfig,
           (long)setConfigRc, (long)setAltRc);
    printf("Endpoints IN:%02x(%u) OUT:%02x(%u)%s\n",
           epIn, (unsigned)epInMax, epOut, (unsigned)epOutMax,
           descriptorsUseful ? " [descriptor]" : " [fallback]");

    printf("Power submit:%ld done:%ld  Special submit:%ld done:%ld\n",
           (long)powerSubmitRc, (long)powerDoneRc,
           (long)specialSubmitRc, (long)specialDoneRc);
    printf("Read submit:%ld done:%ld NAK:%lu  RX:%lu input:%lu cmd:%02x\n\n",
           (long)readSubmitRc, (long)lastReadRc,
           (unsigned long)readNakCount, (unsigned long)rxPackets,
           (unsigned long)inputPackets, lastCmd);

    printf("Buttons: %s\n", buttons);
    printf("btn0:%02x btn1:%02x LT:%4u RT:%4u\n",
           btn0, btn1, (unsigned)lt, (unsigned)rt);
    printf("LX:%6d LY:%6d RX:%6d RY:%6d\n",
           (int)lx, (int)ly, (int)rx, (int)ry);
    printf("Raw:");
    u8 rawToShow = lastRawLen > 24 ? 24 : lastRawLen;
    for (u8 i = 0; i < rawToShow; ++i)
        printf(" %02x", lastPacket[i]);
    printf("\n\n");

    if (inputPackets > 0)
        printf("INPUT LIVE: USB/GIP path is working.\n");
    else if (opened)
        printf("Open is valid even if second Resume is -4. Watch Config/Endpoints/RX.\n");
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

    resetTransportState();
    resetOpenState();
    usbInitRc = USB_Initialize();

    while (1) {
        WPAD_ScanPads();
        if (WPAD_ButtonsDown(0) & WPAD_BUTTON_HOME)
            break;

        if (removed)
            disconnectController();

        if (usbInitRc >= 0 && !opened && (frameCounter % 15) == 0)
            refreshDeviceList();
        if (usbInitRc >= 0)
            openAndConfigure();

        if (opened) {
            processWrite();
            processRead();
        }

        if ((frameCounter % 3) == 0)
            render();

        frameCounter++;
        VIDEO_WaitVSync();
    }

    if (usbFd != -1) {
        s32 closeFd = usbFd;
        USB_CloseDevice(&closeFd);
    }
    USB_Deinitialize();
    return 0;
}
