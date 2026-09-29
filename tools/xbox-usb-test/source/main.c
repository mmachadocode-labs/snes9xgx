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
#define EP_IN      0x82
#define EP_OUT     0x02
#define PKT_SIZE   64
#define MAX_DEVS   32
#define USB_NAK_RC (-7005)

#define ATTACH_WAIT_FRAMES 120
#define RESUME_RETRY_FRAMES 30
#define RESUME_MAX_ATTEMPTS 10

#define WRITE_NONE    0
#define WRITE_POWER   1
#define WRITE_SPECIAL 2
#define WRITE_ACK     3

static usb_device_entry devices[MAX_DEVS] ATTRIBUTE_ALIGN(32);
static u8 inBuf[PKT_SIZE] ATTRIBUTE_ALIGN(32);
static u8 outBuf[PKT_SIZE] ATTRIBUTE_ALIGN(32);
static u8 lastPacket[PKT_SIZE] ATTRIBUTE_ALIGN(32);

static s32 usbInitRc = 999;
static s32 listRc = 999;
static u8 deviceCount = 0;
static s32 interfaceDeviceId[3] = { -1, -1, -1 };
static u32 interfaceToken[3] = { 0, 0, 0 };
static s32 resumeProbe[3] = { 999, 999, 999 };
static s32 targetDeviceId = -1;
static u32 targetToken = 0;
static s32 openRc = 999;
static s32 resumeRc = 999;
static s32 removeHookRc = 999;
static s32 usbFd = -1;
static bool opened = false;
static bool ownershipReady = false;
static bool ownershipFailed = false;
static volatile bool removed = false;
static u32 firstSeenFrame = 0;
static u32 nextResumeFrame = 0;
static int resumeAttempts = 0;

static volatile bool readInFlight = false;
static volatile bool readDone = false;
static volatile s32 readResult = 999;
static s32 readSubmitRc = 999;
static s32 lastReadRc = 999;
static u32 readNakCount = 0;
static u32 rxPackets = 0;
static u32 inputPackets = 0;
static u8 lastRawLen = 0;

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

static bool readingEnabled = false;
static int readRetryDelay = 0;
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

static u8 interfaceNumber(const usb_device_entry *entry)
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

static void resetOwnershipState(void)
{
    targetDeviceId = -1;
    targetToken = 0;
    for (int i = 0; i < 3; ++i) {
        interfaceDeviceId[i] = -1;
        interfaceToken[i] = 0;
        resumeProbe[i] = 999;
    }
    firstSeenFrame = 0;
    nextResumeFrame = 0;
    resumeAttempts = 0;
    openRc = 999;
    resumeRc = 999;
    removeHookRc = 999;
    opened = false;
    ownershipReady = false;
    ownershipFailed = false;
}

static void disconnectController(void)
{
    if (usbFd != -1) {
        s32 closeFd = usbFd;
        USB_CloseDevice(&closeFd);
    }

    usbFd = -1;
    removed = false;
    resetTransportState();
    resetOwnershipState();
}

static s32 submitWrite(int kind, const u8 *packet, u16 len, int seqOverride)
{
    if (!ownershipReady || usbFd == -1 || writeInFlight || !packet || len == 0 || len > PKT_SIZE)
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

    s32 rc = USB_WriteIntrMsgAsync(usbFd, EP_OUT, len, outBuf,
                                   writeCallback, NULL);
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
    if (!ownershipReady || usbFd == -1 || readInFlight)
        return -4;

    memset(inBuf, 0, sizeof(inBuf));
    readDone = false;
    readResult = 999;
    readInFlight = true;
    s32 rc = USB_ReadIntrMsgAsync(usbFd, EP_IN, PKT_SIZE, inBuf,
                                  readCallback, NULL);
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
            if (result < 0 && writeAttempt < 3) {
                scheduleWrite(WRITE_POWER, 12, false);
            } else {
                scheduleWrite(WRITE_SPECIAL, 8, true);
            }
        } else if (kind == WRITE_SPECIAL) {
            specialDoneRc = result;
            if (result < 0 && writeAttempt < 3) {
                scheduleWrite(WRITE_SPECIAL, 12, false);
            } else {
                pendingWriteKind = WRITE_NONE;
                writeAttempt = 0;
                readingEnabled = true;
                readRetryDelay = 2;
            }
        } else if (kind == WRITE_ACK) {
            ackDoneRc = result;
            pendingAck = false;
            writeAttempt = 0;
        }
    }

    if (readingEnabled && pendingAck && pendingWriteKind == WRITE_NONE &&
        !writeInFlight && !writeDone) {
        scheduleWrite(WRITE_ACK, 0, true);
    }

    startScheduledWrite();
}

static void refreshDeviceList(void)
{
    memset(devices, 0, sizeof(devices));
    deviceCount = 0;
    listRc = USB_GetDeviceList(devices, MAX_DEVS, GIP_CLASS, &deviceCount);
    if (listRc < 0)
        return;

    s32 newIds[3] = { -1, -1, -1 };
    u32 newTokens[3] = { 0, 0, 0 };

    for (u8 i = 0; i < deviceCount; ++i) {
        if (devices[i].vid == TARGET_VID && devices[i].pid == TARGET_PID) {
            u8 ifnum = interfaceNumber(&devices[i]);
            if (ifnum < 3) {
                newIds[ifnum] = devices[i].device_id;
                newTokens[ifnum] = devices[i].token;
            }
        }
    }

    if (!ownershipReady && !opened) {
        if (targetDeviceId != newIds[TARGET_IF]) {
            resetOwnershipState();
            firstSeenFrame = frameCounter;
        }

        for (int i = 0; i < 3; ++i) {
            interfaceDeviceId[i] = newIds[i];
            interfaceToken[i] = newTokens[i];
        }
        targetDeviceId = newIds[TARGET_IF];
        targetToken = newTokens[TARGET_IF];

        if (targetDeviceId != -1 && firstSeenFrame == 0)
            firstSeenFrame = frameCounter;
    }
}

static void beginOwnershipProbe(void)
{
    if (targetDeviceId == -1 || ownershipReady || ownershipFailed)
        return;

    if (!opened) {
        if ((frameCounter - firstSeenFrame) < ATTACH_WAIT_FRAMES)
            return;

        s32 candidateFd = -1;
        openRc = USB_OpenDevice(targetDeviceId, TARGET_VID, TARGET_PID, &candidateFd);
        if (openRc < 0)
            return;

        usbFd = candidateFd;
        opened = true;
        nextResumeFrame = frameCounter;
    }

    if (frameCounter < nextResumeFrame)
        return;

    resumeAttempts++;
    for (int i = 0; i < 3; ++i) {
        if (interfaceDeviceId[i] != -1)
            resumeProbe[i] = USB_ResumeDevice(interfaceDeviceId[i]);
    }

    resumeRc = USB_ResumeDevice(usbFd);

    bool anyResumeOk = (resumeRc == 0);
    for (int i = 0; i < 3; ++i)
        anyResumeOk = anyResumeOk || (resumeProbe[i] == 0);

    if (anyResumeOk) {
        ownershipReady = true;
        removeHookRc = USB_DeviceRemovalNotifyAsync(usbFd, removalCallback, NULL);
        scheduleWrite(WRITE_POWER, 4, true);
        return;
    }

    if (resumeAttempts >= RESUME_MAX_ATTEMPTS) {
        ownershipFailed = true;
        return;
    }

    nextResumeFrame = frameCounter + RESUME_RETRY_FRAMES;
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
        return "WAITING FOR LIBOGC ATTACHFINISH";
    if (ownershipFailed)
        return "IOS58 OWNERSHIP FAILED";
    if (!ownershipReady)
        return "PROBING RESUME ON ALL 3 INTERFACES";
    if (!readingEnabled)
        return "GIP INIT";
    if (rxPackets == 0)
        return "READING - WAITING FOR INPUT";
    if (inputPackets == 0)
        return "USB RX - WAITING FOR CMD 20";
    return "INPUT LIVE";
}

static void render(void)
{
    char buttons[160];
    buildButtonString(buttons, sizeof(buttons));

    printf("\x1b[2J\x1b[H");
    printf("XBOX ONE S USB INPUT TEST v1.1 - Wii / libogc2\n");
    printf("Target 045e:02ea  IF:0  GIP ff/47/d0  IN:82 OUT:02\n");
    printf("HOME: exit\n\n");

    printf("Stage: %s\n", stageName());
    printf("USB_Initialize:%ld  GetDeviceList:%ld  count:%u\n",
           (long)usbInitRc, (long)listRc, (unsigned)deviceCount);

    for (int i = 2; i >= 0; --i) {
        if (interfaceDeviceId[i] != -1) {
            printf(" if:%d dev:%ld token:%08lx resume:%ld%s\n",
                   i, (long)interfaceDeviceId[i],
                   (unsigned long)interfaceToken[i],
                   (long)resumeProbe[i], i == TARGET_IF ? "  <GIP>" : "");
        }
    }

    u32 waited = targetDeviceId == -1 ? 0 : frameCounter - firstSeenFrame;
    printf("\nWait:%lu/%d frames  resume attempts:%d/%d\n",
           (unsigned long)waited, ATTACH_WAIT_FRAMES,
           resumeAttempts, RESUME_MAX_ATTEMPTS);
    printf("Selected dev:%ld token:%08lx\n",
           (long)targetDeviceId, (unsigned long)targetToken);
    printf("Open:%ld fd:%ld  Resume(fd):%ld  removal-hook:%ld\n",
           (long)openRc, (long)usbFd, (long)resumeRc, (long)removeHookRc);
    printf("Power  submit:%ld done:%ld  Special submit:%ld done:%ld\n",
           (long)powerSubmitRc, (long)powerDoneRc,
           (long)specialSubmitRc, (long)specialDoneRc);
    printf("Read   submit:%ld done:%ld  NAK:%lu\n",
           (long)readSubmitRc, (long)lastReadRc,
           (unsigned long)readNakCount);
    printf("RX:%lu  input:%lu  cmd:%02x seq:%02x  ACK:%ld/%ld\n\n",
           (unsigned long)rxPackets, (unsigned long)inputPackets,
           lastCmd, lastSeq, (long)ackSubmitRc, (long)ackDoneRc);

    printf("Buttons: %s\n", buttons);
    printf("btn0:%02x btn1:%02x  LT:%4u RT:%4u\n",
           btn0, btn1, (unsigned)lt, (unsigned)rt);
    printf("LX:%6d LY:%6d   RX:%6d RY:%6d\n",
           (int)lx, (int)ly, (int)rx, (int)ry);

    printf("Raw:");
    u8 rawToShow = lastRawLen;
    if (rawToShow > 24)
        rawToShow = 24;
    for (u8 i = 0; i < rawToShow; ++i)
        printf(" %02x", lastPacket[i]);
    printf("\n");

    if (ownershipFailed)
        printf("\nAll delayed Resume probes stayed negative. Photograph this screen.\n");
    else if (ownershipReady && rxPackets == 0)
        printf("\nOwnership passed. Press buttons/move sticks; RX should increase.\n");
    else if (rxPackets > 0)
        printf("\nUSB input is arriving. This path can be ported to Snes9x GX.\n");
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
    resetOwnershipState();
    usbInitRc = USB_Initialize();

    while (1) {
        WPAD_ScanPads();
        if (WPAD_ButtonsDown(0) & WPAD_BUTTON_HOME)
            break;

        if (removed)
            disconnectController();

        if (usbInitRc >= 0 && !ownershipReady && !ownershipFailed &&
            (frameCounter % 15) == 0)
            refreshDeviceList();

        if (usbInitRc >= 0)
            beginOwnershipProbe();

        if (ownershipReady) {
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
