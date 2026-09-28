#ifdef HW_RVL

/*
 * Wired Xbox One input for Snes9x GX.
 *
 * Xbox One S 045e:02ea exposes three IOS58 /dev/usb/ven entries on the test
 * Wii. IOS58 V5 exposes each USB interface as a separate device_id. The raw
 * device-change entry also carries the interface number in usb_device_entry's
 * token field (raw byte 10 => token bits 15..8 on PPC).
 *
 * V23 confirmed interface 0 (FF/47/D0), interrupt IN 0x82 and OUT 0x02, but
 * every GIP interrupt-OUT completion still returned -7005/NAK.
 *
 * V24 fixes an IOS58 V5 requirement that the application transfer buffers
 * passed to /dev/usb/ven live in MEM2 as well as being 32-byte aligned. The
 * previous static .bss buffers were aligned but normally lived in MEM1.
 * Both persistent async buffers are therefore reserved from Arena2/MEM2.
 *
 * USB_OpenDevice() in libogc2 resumes V5 VEN devices internally. The GIP
 * bring-up follows Linux xpad ordering: arm interrupt IN first, then send the
 * output init sequence asynchronously. NAKs are retried without closing the
 * controller. No filesystem logging is used.
 */

#include <gccore.h>
#include <ogc/system.h>
#include <ogc/usb.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>

#define MICROSOFT_VID 0x045e
#define XBOX_ONE_S_PID 0x02ea
#define USB_CLASS_VENDOR_SPECIFIC 0xff
#define MAX_USB_DEVICES 24
#define MAX_REPORT_SIZE 64
#define POLL_INTERVAL_FRAMES 60
#define RETRY_INTERVAL_FRAMES 15
#define STICK_THRESHOLD 16384
#define TRIGGER_THRESHOLD 128
#define USB_NAK_RC (-7005)
#define XBOX_GIP_INTERFACE 0
#define XBOX_GIP_IN_EP 0x82
#define XBOX_GIP_OUT_EP 0x02

#define GIP_CMD_INPUT       0x20
#define GIP_CMD_VIRTUAL_KEY 0x07

typedef struct {
    u16 pid;
    const char *name;
} xbox_profile;

static const xbox_profile profiles[] = {
    { 0x02d1, "Xbox One" },
    { 0x02dd, "Xbox One (2015)" },
    { 0x02e3, "Xbox One Elite" },
    { XBOX_ONE_S_PID, "Xbox One S" },
    { 0x0b12, "Xbox Elite 2" },
    { 0x0b13, "Xbox Series X/S" },
};

static bool initialized = false;
static volatile bool active = false;
static volatile bool closePending = false;
static volatile s32 asyncError = 0;
static volatile s32 deviceFd = -1; /* negative V5 ids are valid; -1 is sentinel */
static const xbox_profile *activeProfile = NULL;
static u8 endpointIn = 0;
static u8 endpointOut = 0;
static u16 reportLength = MAX_REPORT_SIZE;
static volatile u32 heldButtons = 0;
static volatile bool guidePressed = false;
static int pollCountdown = 0;
static char statusText[192] = "not found";

/* RAM-only diagnostics shown in Credits. */
static s32 lastFd = -1;
static u8 selectedListIndex = 0xff;
static u8 selectedInterface = 0xff;
static u8 selectedAltCount = 0xff;
static u32 selectedToken = 0;
static u8 matchingEntries = 0;
static s32 lastReadQueueRc = 999;
static volatile s32 lastWriteCompletionRc = 999;
static s32 lastWriteSubmitRc = 999;
static volatile u32 rxPackets = 0;
static volatile u8 lastRxCommand = 0;
static u32 initRetries = 0;

/*
 * /dev/usb/ven requires application transfer buffers in MEM2 and 32-byte
 * aligned. Reserve one permanent 128-byte Arena2 block and split it into two
 * 64-byte buffers. Arena allocations are intentionally kept for app lifetime.
 */
static u8 *usbBufferBlock = NULL;
static u8 *reportBuffer = NULL;
static u8 *outputBuffer = NULL;

/* xpad-style output init state. */
static volatile bool writeInFlight = false;
static volatile bool writeResultPending = false;
static volatile s32 pendingWriteResult = 0;
static int initStage = 0; /* 0 power, 1 One-S, 2 LED, 3 auth, 4 done */
static u8 initSerial = 0;
static int initRetryCountdown = 0;

static bool readRetryPending = false;
static int readRetryCountdown = 0;

static bool allocateUsbBuffers(void)
{
    if (reportBuffer && outputBuffer)
        return true;

    usbBufferBlock = (u8 *)SYS_AllocArenaMem2Lo(MAX_REPORT_SIZE * 2, 32);
    if (!usbBufferBlock) {
        snprintf(statusText, sizeof(statusText), "MEM2 USB buffer alloc failed");
        return false;
    }

    reportBuffer = usbBufferBlock;
    outputBuffer = usbBufferBlock + MAX_REPORT_SIZE;
    memset(reportBuffer, 0, MAX_REPORT_SIZE);
    memset(outputBuffer, 0, MAX_REPORT_SIZE);
    return true;
}

static const xbox_profile *findProfile(u16 vid, u16 pid)
{
    if (vid != MICROSOFT_VID)
        return NULL;

    for (unsigned i = 0; i < sizeof(profiles) / sizeof(profiles[0]); ++i) {
        if (profiles[i].pid == pid)
            return &profiles[i];
    }
    return NULL;
}

/*
 * IOS58 V5 GetDeviceChange entry bytes 8..11 are stored in entry->token:
 *   bytes 8..9  = device number
 *   byte 10     = interface number
 *   byte 11     = number of alternate settings
 * Wii/PPC is big-endian, hence interface is token bits 15..8.
 */
static u8 entryInterfaceNumber(const usb_device_entry *entry)
{
    return (u8)((entry->token >> 8) & 0xff);
}

static u8 entryAltCount(const usb_device_entry *entry)
{
    return (u8)(entry->token & 0xff);
}

static s16 readS16LE(const u8 *p)
{
    return (s16)((u16)p[0] | ((u16)p[1] << 8));
}

static u16 readU16LE(const u8 *p)
{
    return (u16)p[0] | ((u16)p[1] << 8);
}

static void parseXboxOneReport(const u8 *d, u16 len)
{
    if (!d || len < 5)
        return;

    lastRxCommand = d[0];
    rxPackets++;

    if (d[0] == GIP_CMD_VIRTUAL_KEY) {
        guidePressed = (d[4] & 0x01) != 0;
        return;
    }

    if (d[0] != GIP_CMD_INPUT || len < 18)
        return;

    u32 buttons = 0;

    if (d[4] & 0x04) buttons |= PAD_BUTTON_START;
    if (d[4] & 0x08) buttons |= PAD_TRIGGER_Z;

    if (d[4] & 0x10) buttons |= PAD_BUTTON_B; /* Xbox A -> SNES B */
    if (d[4] & 0x20) buttons |= PAD_BUTTON_A; /* Xbox B -> SNES A */
    if (d[4] & 0x40) buttons |= PAD_BUTTON_Y; /* Xbox X -> SNES Y */
    if (d[4] & 0x80) buttons |= PAD_BUTTON_X; /* Xbox Y -> SNES X */

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
    s16 ly = (s16)~readS16LE(&d[12]);
    s16 rx = readS16LE(&d[14]);
    s16 ry = (s16)~readS16LE(&d[16]);

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

static s32 readCallback(s32 result, void *userdata)
{
    (void)userdata;

    if (result < 0) {
        if (result == USB_NAK_RC) {
            readRetryPending = true;
            readRetryCountdown = RETRY_INTERVAL_FRAMES;
        } else {
            asyncError = result;
            closePending = true;
        }
        return 0;
    }

    if (result > 0)
        parseXboxOneReport(reportBuffer, (u16)result);

    if (active && deviceFd != -1) {
        s32 rc = USB_ReadIntrMsgAsync(deviceFd, endpointIn, reportLength,
                                      reportBuffer, readCallback, NULL);
        lastReadQueueRc = rc;
        if (rc < 0) {
            if (rc == USB_NAK_RC) {
                readRetryPending = true;
                readRetryCountdown = RETRY_INTERVAL_FRAMES;
            } else {
                asyncError = rc;
                closePending = true;
            }
        }
    }
    return 0;
}

static s32 writeCallback(s32 result, void *userdata)
{
    (void)userdata;
    lastWriteCompletionRc = result;
    pendingWriteResult = result;
    writeInFlight = false;
    writeResultPending = true;
    return 0;
}

static void closeController(void)
{
    s32 fd = deviceFd;
    active = false;
    deviceFd = -1;
    writeInFlight = false;
    writeResultPending = false;
    readRetryPending = false;

    if (fd != -1)
        USB_CloseDevice(&fd);

    activeProfile = NULL;
    endpointIn = endpointOut = 0;
    heldButtons = 0;
    guidePressed = false;
    closePending = false;
}

static bool prepareInitPacket(void)
{
    static const u8 powerOn[] = { 0x05, 0x20, 0x00, 0x01, 0x00 };
    static const u8 oneSInit[] = { 0x05, 0x20, 0x00, 0x0f, 0x06 };
    static const u8 ledOn[] = { 0x0a, 0x20, 0x00, 0x03, 0x00, 0x01, 0x14 };
    static const u8 authDone[] = { 0x06, 0x20, 0x00, 0x02, 0x01, 0x00 };

    const u8 *data = NULL;
    u16 len = 0;

    while (initStage < 4) {
        switch (initStage) {
            case 0:
                data = powerOn;
                len = sizeof(powerOn);
                break;
            case 1:
                if (activeProfile && activeProfile->pid == XBOX_ONE_S_PID) {
                    data = oneSInit;
                    len = sizeof(oneSInit);
                } else {
                    initStage++;
                    continue;
                }
                break;
            case 2:
                data = ledOn;
                len = sizeof(ledOn);
                break;
            case 3:
                data = authDone;
                len = sizeof(authDone);
                break;
        }
        break;
    }

    if (!data || !len || !outputBuffer)
        return false;

    memset(outputBuffer, 0, MAX_REPORT_SIZE);
    memcpy(outputBuffer, data, len);
    outputBuffer[2] = initSerial;
    return true;
}

static void submitCurrentInitPacket(void)
{
    if (!active || deviceFd == -1 || !endpointOut || !outputBuffer ||
        writeInFlight || initStage >= 4)
        return;

    if (!prepareInitPacket())
        return;

    u16 len = (initStage == 2) ? 7 : (initStage == 3) ? 6 : 5;
    s32 rc = USB_WriteIntrMsgAsync(deviceFd, endpointOut, len,
                                   outputBuffer, writeCallback, NULL);
    lastWriteSubmitRc = rc;

    if (rc >= 0) {
        writeInFlight = true;
    } else {
        lastWriteCompletionRc = rc;
        pendingWriteResult = rc;
        writeResultPending = true;
    }
}

static void processInitState(void)
{
    if (!active)
        return;

    if (writeResultPending) {
        s32 rc = pendingWriteResult;
        writeResultPending = false;

        if (rc >= 0) {
            initStage++;
            initSerial++;
            initRetries = 0;
            initRetryCountdown = 1;
        } else {
            initRetries++;
            initRetryCountdown = RETRY_INTERVAL_FRAMES;
        }
    }

    if (writeInFlight || initStage >= 4)
        return;

    if (initRetryCountdown > 0) {
        initRetryCountdown--;
        return;
    }

    submitCurrentInitPacket();
}

static void processReadRetry(void)
{
    if (!active || !readRetryPending || deviceFd == -1 || !reportBuffer)
        return;

    if (readRetryCountdown > 0) {
        readRetryCountdown--;
        return;
    }

    memset(reportBuffer, 0, MAX_REPORT_SIZE);
    s32 rc = USB_ReadIntrMsgAsync(deviceFd, endpointIn, reportLength,
                                  reportBuffer, readCallback, NULL);
    lastReadQueueRc = rc;

    if (rc >= 0) {
        readRetryPending = false;
    } else if (rc == USB_NAK_RC) {
        readRetryCountdown = RETRY_INTERVAL_FRAMES;
    } else {
        asyncError = rc;
        closePending = true;
    }
}

static void updateStatus(void)
{
    if (!active)
        return;

    snprintf(statusText, sizeof(statusText),
             "x1s e:%u if:%u M2:%08x/%08x q:%d w:%d/%d s:%d n:%u rx:%u c:%02x",
             (unsigned)selectedListIndex, (unsigned)selectedInterface,
             (unsigned)(u32)reportBuffer, (unsigned)(u32)outputBuffer,
             lastReadQueueRc, lastWriteSubmitRc,
             (s32)lastWriteCompletionRc, initStage,
             (unsigned)initRetries, (unsigned)rxPackets,
             (unsigned)lastRxCommand);
}

static bool openSelectedEntry(const usb_device_entry *entry,
                              const xbox_profile *profile,
                              u8 interfaceNumber, u8 listIndex)
{
    s32 fd = -1;
    s32 rc = USB_OpenDevice(entry->device_id, entry->vid, entry->pid, &fd);
    lastFd = fd;
    if (rc < 0) {
        snprintf(statusText, sizeof(statusText),
                 "open:%d e:%u if:%u t:%08x", rc,
                 (unsigned)listIndex, (unsigned)interfaceNumber,
                 (unsigned)entry->token);
        return false;
    }

    deviceFd = fd;
    activeProfile = profile;
    endpointIn = XBOX_GIP_IN_EP;
    endpointOut = XBOX_GIP_OUT_EP;
    reportLength = MAX_REPORT_SIZE;
    selectedListIndex = listIndex;
    selectedInterface = interfaceNumber;
    selectedAltCount = entryAltCount(entry);
    selectedToken = entry->token;
    heldButtons = 0;
    guidePressed = false;
    asyncError = 0;
    closePending = false;
    active = true;

    lastReadQueueRc = 999;
    lastWriteSubmitRc = 999;
    lastWriteCompletionRc = 999;
    rxPackets = 0;
    lastRxCommand = 0;
    initRetries = 0;
    initStage = 0;
    initSerial = 0;
    initRetryCountdown = 0;
    writeInFlight = false;
    writeResultPending = false;
    readRetryPending = false;
    readRetryCountdown = 0;

    /* Linux xpad arms input before starting the Xbox One output init. */
    memset(reportBuffer, 0, MAX_REPORT_SIZE);
    rc = USB_ReadIntrMsgAsync(deviceFd, endpointIn, reportLength,
                              reportBuffer, readCallback, NULL);
    lastReadQueueRc = rc;
    if (rc < 0) {
        if (rc == USB_NAK_RC) {
            readRetryPending = true;
            readRetryCountdown = RETRY_INTERVAL_FRAMES;
        } else {
            snprintf(statusText, sizeof(statusText),
                     "readq:%d e:%u if:%u t:%08x", rc,
                     (unsigned)listIndex, (unsigned)interfaceNumber,
                     (unsigned)entry->token);
            closeController();
            return false;
        }
    }

    /* Keep input alive even if output initially NAKs. */
    submitCurrentInitPacket();
    updateStatus();
    return true;
}

static bool scanVendorClass(void)
{
    usb_device_entry *entries = (usb_device_entry *)memalign(
        32, sizeof(usb_device_entry) * MAX_USB_DEVICES);
    if (!entries) {
        snprintf(statusText, sizeof(statusText), "alloc list failed");
        return false;
    }

    memset(entries, 0, sizeof(usb_device_entry) * MAX_USB_DEVICES);
    u8 count = 0;
    s32 rc = USB_GetDeviceList(entries, MAX_USB_DEVICES,
                               USB_CLASS_VENDOR_SPECIFIC, &count);
    if (rc < 0) {
        snprintf(statusText, sizeof(statusText), "list:%d", rc);
        free(entries);
        return false;
    }

    matchingEntries = 0;
    u32 lastMatchingToken = 0;
    u8 lastMatchingInterface = 0xff;

    for (u8 i = 0; i < count; ++i) {
        const xbox_profile *profile = findProfile(entries[i].vid, entries[i].pid);
        if (!profile)
            continue;

        matchingEntries++;
        u8 interfaceNumber = entryInterfaceNumber(&entries[i]);
        lastMatchingToken = entries[i].token;
        lastMatchingInterface = interfaceNumber;

        /* Xbox One S GIP data interface is interface 0 (FF/47/D0). */
        if (entries[i].pid == XBOX_ONE_S_PID &&
            interfaceNumber != XBOX_GIP_INTERFACE)
            continue;

        if (openSelectedEntry(&entries[i], profile, interfaceNumber, i)) {
            free(entries);
            return true;
        }
    }

    if (matchingEntries)
        snprintf(statusText, sizeof(statusText),
                 "Xbox ifs:%u no if0 lastif:%u t:%08x",
                 (unsigned)matchingEntries,
                 (unsigned)lastMatchingInterface,
                 (unsigned)lastMatchingToken);
    else
        snprintf(statusText, sizeof(statusText), "not found (%u USB)", count);

    free(entries);
    return false;
}

static void initializeDriver(void)
{
    if (initialized)
        return;

    initialized = true;
    active = false;
    deviceFd = -1;
    heldButtons = 0;
    closePending = false;
    asyncError = 0;

    if (!allocateUsbBuffers())
        return;

    s32 rc = USB_Initialize();
    if (rc < 0) {
        snprintf(statusText, sizeof(statusText), "USB init:%d", rc);
        return;
    }

    scanVendorClass();
    pollCountdown = POLL_INTERVAL_FRAMES;
}

void XBOXONE_ScanPads(void)
{
    initializeDriver();

    if (closePending) {
        s32 err = asyncError;
        closeController();
        snprintf(statusText, sizeof(statusText),
                 "usbcb:%d fd:%d q:%d w:%d/%d",
                 err, lastFd, lastReadQueueRc,
                 lastWriteSubmitRc, (s32)lastWriteCompletionRc);
        pollCountdown = 0;
    }

    if (active) {
        processReadRetry();
        processInitState();
        updateStatus();
        return;
    }

    if (--pollCountdown > 0)
        return;

    pollCountdown = POLL_INTERVAL_FRAMES;
    scanVendorClass();
}

u32 XBOXONE_ButtonsHeld(int chan)
{
    if (!initialized)
        initializeDriver();

    if (!active || chan != 0)
        return 0;

    return heldButtons;
}

char *XBOXONE_Status(void)
{
    if (!initialized)
        initializeDriver();
    updateStatus();
    return statusText;
}

#endif