#ifdef HW_RVL

/*
 * Wired Xbox One input for Snes9x GX.
 *
 * USB transport/lifetime handling is adapted from Mayo1970/ioQuake3-wii's
 * code/input/wii_usb_hid.c (GPLv2). IOS IPC buffers are 32-byte aligned,
 * hotplug is polled, and the async read callback never performs filesystem
 * I/O or closes/reopens the USB device.
 *
 * Xbox One S (045e:02ea) uses a descriptor-less fast path on IOS58 because
 * USB_GetDescriptors() returns IPC_EINVAL (-4) for the V5 vendor device on
 * the test Wii. Its known GIP endpoints are interrupt IN 0x82 and OUT 0x02.
 *
 * Diagnostic build: early messages are buffered in RAM because the Wii input
 * driver is initialized before the filesystem/SD card is mounted. Once SD is
 * available, the main thread writes the buffered trace to xbox-usb.log.
 */

#include <gccore.h>
#include <ogc/usb.h>
#include <malloc.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define MICROSOFT_VID 0x045e
#define XBOX_ONE_S_PID 0x02ea
#define USB_CLASS_VENDOR_SPECIFIC 0xff
#define MAX_USB_DEVICES 16
#define MAX_REPORT_SIZE 64
#define POLL_INTERVAL_FRAMES 60
#define STICK_THRESHOLD 16384
#define TRIGGER_THRESHOLD 128

#define GIP_CMD_INPUT       0x20
#define GIP_CMD_VIRTUAL_KEY 0x07

#define DIAG_EVENT_COUNT 32
#define DIAG_CAPTURE_BYTES 32
#define DIAG_PENDING_BYTES 16384

typedef struct {
    u16 pid;
    const char *name;
} xbox_profile;

typedef struct {
    s32 result;
    s32 requeue;
    u8 length;
    u8 data[DIAG_CAPTURE_BYTES];
} diag_event;

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
/* IOS58 V5 vendor device IDs are valid negative numbers. -1 is our sentinel. */
static volatile s32 deviceFd = -1;
static const xbox_profile *activeProfile = NULL;
static u8 endpointIn = 0;
static u8 endpointOut = 0;
static u16 reportLength = MAX_REPORT_SIZE;
static volatile u32 heldButtons = 0;
static volatile bool guidePressed = false;
static int pollCountdown = 0;
static char statusText[112] = "not found";

/* IOS DMAs directly into this buffer. Keep it static and 32-byte aligned. */
static u8 ATTRIBUTE_ALIGN(32) reportBuffer[MAX_REPORT_SIZE];

/* Diagnostic file is used only by the main thread. */
static FILE *diagFile = NULL;
static u32 diagSequence = 0;
static u32 diagPositiveReadsLogged = 0;
static char diagPending[DIAG_PENDING_BYTES];
static size_t diagPendingLen = 0;
static u32 diagPendingDropped = 0;
static char diagOpenedPath[64] = "RAM only";

/* Callback -> main-thread ring buffer. No file I/O from the IOS callback. */
static diag_event diagEvents[DIAG_EVENT_COUNT];
static volatile u32 diagEventWrite = 0;
static volatile u32 diagEventRead = 0;
static volatile u32 diagEventsDropped = 0;

static void diagQueueBytes(const char *data, size_t len)
{
    if (!data || !len)
        return;

    if (len > sizeof(diagPending) - diagPendingLen) {
        size_t room = sizeof(diagPending) - diagPendingLen;
        if (room) {
            memcpy(diagPending + diagPendingLen, data, room);
            diagPendingLen += room;
        }
        diagPendingDropped++;
        return;
    }

    memcpy(diagPending + diagPendingLen, data, len);
    diagPendingLen += len;
}

/*
 * The input driver comes up before WiiFileSystemDriver::init(), so fopen()
 * commonly fails during the first Xbox scan. Do not latch that failure.
 * Retry from the normal main-thread polling path until SD becomes available.
 */
static bool diagTryOpen(void)
{
    if (diagFile)
        return true;

    const char *paths[] = {
        "sd:/snes9xgx/xbox-usb.log",
        "sd:/xbox-usb.log",
        "usb:/snes9xgx/xbox-usb.log",
        "usb:/xbox-usb.log"
    };

    for (unsigned i = 0; i < sizeof(paths) / sizeof(paths[0]); ++i) {
        FILE *f = fopen(paths[i], "w");
        if (!f)
            continue;

        /* Diagnostic build: unbuffered writes make every completed line durable. */
        setvbuf(f, NULL, _IONBF, 0);

        if (diagPendingLen) {
            size_t n = fwrite(diagPending, 1, diagPendingLen, f);
            if (n != diagPendingLen) {
                fclose(f);
                continue;
            }
        }

        diagFile = f;
        strncpy(diagOpenedPath, paths[i], sizeof(diagOpenedPath) - 1);
        diagOpenedPath[sizeof(diagOpenedPath) - 1] = 0;
        diagPendingLen = 0;

        fprintf(diagFile, "[diag] file opened: %s\n", diagOpenedPath);
        if (diagPendingDropped)
            fprintf(diagFile, "[diag] RAM log overflow before mount: dropped=%u\n",
                    (unsigned)diagPendingDropped);
        return true;
    }

    return false;
}

static void diagLog(const char *fmt, ...)
{
    char line[512];
    int prefix = snprintf(line, sizeof(line), "[%06u] ", (unsigned)diagSequence++);
    if (prefix < 0)
        return;
    if ((size_t)prefix >= sizeof(line))
        prefix = sizeof(line) - 1;

    va_list ap;
    va_start(ap, fmt);
    int body = vsnprintf(line + prefix, sizeof(line) - (size_t)prefix, fmt, ap);
    va_end(ap);

    size_t len;
    if (body < 0)
        len = (size_t)prefix;
    else {
        size_t wanted = (size_t)prefix + (size_t)body;
        len = wanted < sizeof(line) ? wanted : sizeof(line) - 1;
    }

    if (len + 1 < sizeof(line))
        line[len++] = '\n';
    else {
        line[sizeof(line) - 2] = '\n';
        len = sizeof(line) - 1;
    }

    if (diagTryOpen()) {
        size_t n = fwrite(line, 1, len, diagFile);
        if (n == len)
            return;

        /* Storage disappeared or write failed. Preserve future lines in RAM. */
        fclose(diagFile);
        diagFile = NULL;
        strncpy(diagOpenedPath, "RAM after write failure", sizeof(diagOpenedPath) - 1);
        diagOpenedPath[sizeof(diagOpenedPath) - 1] = 0;
    }

    diagQueueBytes(line, len);
}

static void drainDiagEvents(void)
{
    u32 dropped = diagEventsDropped;
    if (dropped) {
        diagEventsDropped = 0;
        diagLog("CALLBACK queue overflow: dropped=%u", (unsigned)dropped);
    }

    while (diagEventRead != diagEventWrite) {
        u32 r = diagEventRead;
        diag_event ev = diagEvents[r % DIAG_EVENT_COUNT];
        diagEventRead = r + 1;

        if (ev.result > 0 && diagPositiveReadsLogged >= 64)
            continue;

        if (ev.result > 0)
            diagPositiveReadsLogged++;

        char hex[(DIAG_CAPTURE_BYTES * 3) + 1];
        hex[0] = 0;
        size_t pos = 0;
        for (u8 i = 0; i < ev.length && pos + 4 < sizeof(hex); ++i) {
            int n = snprintf(hex + pos, sizeof(hex) - pos, "%02x%s",
                             ev.data[i], (i + 1 < ev.length) ? " " : "");
            if (n < 0)
                break;
            pos += (size_t)n;
        }

        if (ev.result > 0)
            diagLog("READ callback result=%d requeue=%d bytes[%u]=%s",
                    ev.result, ev.requeue, ev.length, hex);
        else
            diagLog("READ callback result=%d requeue=%d", ev.result, ev.requeue);
    }
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

    if (d[0] == GIP_CMD_VIRTUAL_KEY) {
        guidePressed = (d[4] & 0x01) != 0;
        return;
    }

    if (d[0] != GIP_CMD_INPUT || len < 18)
        return;

    u32 buttons = 0;

    if (d[4] & 0x04) buttons |= PAD_BUTTON_START;
    if (d[4] & 0x08) buttons |= PAD_TRIGGER_Z;

    /* Preserve Snes9x GX's Nintendo-style face-button placement. */
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

    u32 w = diagEventWrite;
    diag_event *ev = NULL;
    if ((w - diagEventRead) < DIAG_EVENT_COUNT) {
        ev = &diagEvents[w % DIAG_EVENT_COUNT];
        ev->result = result;
        ev->requeue = 0;
        ev->length = 0;

        if (result > 0) {
            u32 copyLen = (u32)result;
            if (copyLen > DIAG_CAPTURE_BYTES)
                copyLen = DIAG_CAPTURE_BYTES;
            ev->length = (u8)copyLen;
            memcpy(ev->data, reportBuffer, copyLen);
        }
    } else {
        diagEventsDropped++;
    }

    if (result < 0) {
        asyncError = result;
        closePending = true;
        if (ev)
            diagEventWrite = w + 1;
        return 0;
    }

    if (result > 0)
        parseXboxOneReport(reportBuffer, (u16)result);

    s32 requeue = 0;
    if (active && deviceFd != -1) {
        requeue = USB_ReadIntrMsgAsync(deviceFd, endpointIn, reportLength,
                                       reportBuffer, readCallback, NULL);
        if (requeue < 0) {
            asyncError = requeue;
            closePending = true;
        }
    }

    if (ev) {
        ev->requeue = requeue;
        diagEventWrite = w + 1;
    }
    return 0;
}

static void closeController(void)
{
    s32 fd = deviceFd;
    active = false;
    deviceFd = -1;

    if (fd != -1) {
        s32 tmp = fd;
        s32 rc = USB_CloseDevice(&tmp);
        diagLog("USB_CloseDevice fd=%d -> %d", fd, rc);
    }

    activeProfile = NULL;
    endpointIn = endpointOut = 0;
    heldButtons = 0;
    guidePressed = false;
    closePending = false;
}

/* Prefer the Xbox GIP FF/47/D0 interface, then first interrupt IN/OUT pair. */
static bool findEndpoints(usb_devdesc *dd, u8 *inEp, u8 *outEp)
{
    if (!dd || !inEp || !outEp)
        return false;

    *inEp = 0;
    *outEp = 0;

    for (u8 c = 0; c < dd->bNumConfigurations; ++c) {
        usb_configurationdesc *cd = &dd->configurations[c];
        for (u8 i = 0; i < cd->bNumInterfaces; ++i) {
            usb_interfacedesc *id = &cd->interfaces[i];
            if (id->bInterfaceClass != 0xff ||
                id->bInterfaceSubClass != 0x47 ||
                id->bInterfaceProtocol != 0xd0)
                continue;

            for (u8 e = 0; e < id->bNumEndpoints; ++e) {
                usb_endpointdesc *ep = &id->endpoints[e];
                if ((ep->bmAttributes & 0x03) != USB_ENDPOINT_INTERRUPT)
                    continue;
                if (ep->bEndpointAddress & USB_ENDPOINT_IN)
                    *inEp = ep->bEndpointAddress;
                else
                    *outEp = ep->bEndpointAddress;
            }
            if (*inEp && *outEp)
                return true;
        }
    }

    *inEp = *outEp = 0;
    for (u8 c = 0; c < dd->bNumConfigurations; ++c) {
        usb_configurationdesc *cd = &dd->configurations[c];
        for (u8 i = 0; i < cd->bNumInterfaces; ++i) {
            usb_interfacedesc *id = &cd->interfaces[i];
            for (u8 e = 0; e < id->bNumEndpoints; ++e) {
                usb_endpointdesc *ep = &id->endpoints[e];
                if ((ep->bmAttributes & 0x03) != USB_ENDPOINT_INTERRUPT)
                    continue;
                if ((ep->bEndpointAddress & USB_ENDPOINT_IN) && !*inEp)
                    *inEp = ep->bEndpointAddress;
                else if (!(ep->bEndpointAddress & USB_ENDPOINT_IN) && !*outEp)
                    *outEp = ep->bEndpointAddress;
            }
            if (*inEp && *outEp)
                return true;
        }
    }

    return *inEp != 0;
}

static s32 writePacketAligned(s32 fd, u8 outEp, const u8 *data, u8 len)
{
    if (!outEp || !data || !len || len > 32)
        return -1;

    u8 *out = (u8 *)memalign(32, 32);
    if (!out)
        return -1;

    memset(out, 0, 32);
    memcpy(out, data, len);
    s32 rc = USB_WriteIntrMsg(fd, outEp, len, out);
    free(out);
    return rc;
}

static s32 retryWritePacket(s32 fd, u8 outEp, const char *label,
                            const u8 *data, u8 len, int attempts)
{
    s32 rc = -1;
    for (int i = 0; i < attempts; ++i) {
        rc = writePacketAligned(fd, outEp, data, len);
        diagLog("WRITE %s attempt=%d ep=%02x len=%u -> %d",
                label, i + 1, outEp, len, rc);
        if (rc >= 0)
            break;
        usleep(25000);
    }
    return rc;
}

static bool tryOpen(const usb_device_entry *entry)
{
    const xbox_profile *profile = findProfile(entry->vid, entry->pid);
    if (!profile)
        return false;

    diagLog("TRY device_id=%d vid=%04x pid=%04x name=%s",
            entry->device_id, entry->vid, entry->pid, profile->name);

    s32 fd = -1;
    s32 rc = USB_OpenDevice(entry->device_id, entry->vid, entry->pid, &fd);
    diagLog("USB_OpenDevice device_id=%d -> rc=%d fd=%d",
            entry->device_id, rc, fd);
    if (rc < 0) {
        snprintf(statusText, sizeof(statusText), "open:%d %04x:%04x",
                 rc, entry->vid, entry->pid);
        return false;
    }

    u8 inEp = 0;
    u8 outEp = 0;
    bool directPath = entry->vid == MICROSOFT_VID && entry->pid == XBOX_ONE_S_PID;

    if (directPath) {
        inEp = 0x82;
        outEp = 0x02;
        diagLog("endpoint mode=direct IN=%02x OUT=%02x packet=%u",
                inEp, outEp, MAX_REPORT_SIZE);
    } else {
        usb_devdesc *dd = (usb_devdesc *)memalign(32, sizeof(usb_devdesc));
        if (!dd) {
            diagLog("descriptor alloc failed");
            USB_CloseDevice(&fd);
            return false;
        }
        memset(dd, 0, sizeof(*dd));

        rc = USB_GetDescriptors(fd, dd);
        diagLog("USB_GetDescriptors fd=%d -> %d", fd, rc);
        if (rc < 0) {
            snprintf(statusText, sizeof(statusText), "desc:%d %04x:%04x",
                     rc, entry->vid, entry->pid);
            USB_CloseDevice(&fd);
            free(dd);
            return false;
        }

        bool endpointsOk = findEndpoints(dd, &inEp, &outEp);
        diagLog("descriptor endpoints ok=%d IN=%02x OUT=%02x",
                endpointsOk ? 1 : 0, inEp, outEp);
        USB_FreeDescriptors(dd);
        free(dd);

        if (!endpointsOk || !inEp || !outEp) {
            snprintf(statusText, sizeof(statusText), "no endpoints %04x:%04x",
                     entry->vid, entry->pid);
            USB_CloseDevice(&fd);
            return false;
        }
    }

    s32 resumeRc = USB_ResumeDevice(fd);
    diagLog("USB_ResumeDevice fd=%d -> %d", fd, resumeRc);
    usleep(150000);

    s32 clearInRc = USB_ClearHalt(fd, inEp);
    s32 clearOutRc = USB_ClearHalt(fd, outEp);
    diagLog("USB_ClearHalt IN=%02x -> %d; OUT=%02x -> %d",
            inEp, clearInRc, outEp, clearOutRc);

    deviceFd = fd;
    activeProfile = profile;
    endpointIn = inEp;
    endpointOut = outEp;
    reportLength = MAX_REPORT_SIZE;
    heldButtons = 0;
    guidePressed = false;
    asyncError = 0;
    closePending = false;
    active = true;
    memset(reportBuffer, 0, sizeof(reportBuffer));

    s32 readQueueRc = USB_ReadIntrMsgAsync(deviceFd, endpointIn, reportLength,
                                           reportBuffer, readCallback, NULL);
    diagLog("USB_ReadIntrMsgAsync fd=%d ep=%02x len=%u -> %d",
            fd, endpointIn, reportLength, readQueueRc);
    if (readQueueRc < 0) {
        snprintf(statusText, sizeof(statusText), "direct readq:%d ep:%02x/%02x",
                 readQueueRc, endpointIn, endpointOut);
        closeController();
        return false;
    }

    static const u8 powerOn[] = { 0x05, 0x20, 0x00, 0x01, 0x00 };
    static const u8 oneSInit[] = { 0x05, 0x20, 0x00, 0x0f, 0x06 };

    s32 powerRc = retryWritePacket(fd, outEp, "power-on",
                                   powerOn, sizeof(powerOn), 12);
    s32 oneSRc = 0;
    if (entry->pid == XBOX_ONE_S_PID)
        oneSRc = retryWritePacket(fd, outEp, "one-s-init",
                                  oneSInit, sizeof(oneSInit), 6);

    diagLog("OPEN COMPLETE fd=%d resume=%d clear=%d/%d readq=%d power=%d oneS=%d",
            fd, resumeRc, clearInRc, clearOutRc, readQueueRc, powerRc, oneSRc);

    snprintf(statusText, sizeof(statusText),
             "connected diag r:%d q:%d w:%d/%d ep:%02x/%02x",
             resumeRc, readQueueRc, powerRc, oneSRc, endpointIn, endpointOut);
    return true;
}

static bool scanVendorClass(void)
{
    usb_device_entry *entries = (usb_device_entry *)memalign(
        32, sizeof(usb_device_entry) * MAX_USB_DEVICES);
    if (!entries) {
        snprintf(statusText, sizeof(statusText), "alloc list failed");
        diagLog("USB device list alloc failed");
        return false;
    }

    memset(entries, 0, sizeof(usb_device_entry) * MAX_USB_DEVICES);
    u8 count = 0;
    s32 rc = USB_GetDeviceList(entries, MAX_USB_DEVICES,
                               USB_CLASS_VENDOR_SPECIFIC, &count);
    diagLog("USB_GetDeviceList class=ff -> rc=%d count=%u", rc, count);
    if (rc < 0) {
        snprintf(statusText, sizeof(statusText), "list:%d", rc);
        free(entries);
        return false;
    }

    bool sawSupported = false;
    for (u8 i = 0; i < count; ++i) {
        diagLog("LIST[%u] device_id=%d vid=%04x pid=%04x",
                i, entries[i].device_id, entries[i].vid, entries[i].pid);

        if (!findProfile(entries[i].vid, entries[i].pid))
            continue;

        sawSupported = true;
        if (tryOpen(&entries[i])) {
            free(entries);
            return true;
        }
    }

    if (!sawSupported)
        snprintf(statusText, sizeof(statusText), "not found (%u USB)", count);

    free(entries);
    return false;
}

static void initializeDriver(void)
{
    if (initialized)
        return;

    /* These lines will stay in RAM until the SD filesystem is mounted. */
    diagLog("===== Xbox USB diagnostic session (IOS %u) =====",
            (unsigned)IOS_GetVersion());
    diagLog("driver init begin");

    initialized = true;
    active = false;
    deviceFd = -1;
    heldButtons = 0;
    closePending = false;
    asyncError = 0;
    diagEventRead = diagEventWrite = 0;
    diagEventsDropped = 0;

    s32 rc = USB_Initialize();
    diagLog("USB_Initialize -> %d", rc);
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

    /* Filesystem initialization happens after input initialization. Keep
     * retrying here from the normal main thread until SD becomes writable. */
    diagTryOpen();
    drainDiagEvents();

    if (closePending) {
        s32 err = asyncError;
        diagLog("closePending asyncError=%d", err);
        closeController();
        snprintf(statusText, sizeof(statusText), "disconnected read:%d", err);
        pollCountdown = 0;
    }

    if (active)
        return;

    if (--pollCountdown > 0)
        return;

    pollCountdown = POLL_INTERVAL_FRAMES;
    diagLog("poll rescan");
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
    diagTryOpen();
    drainDiagEvents();

    /* If the SD still is not mounted, make that visible in Credits. */
    if (!diagFile && strstr(statusText, "diag") == NULL) {
        static char statusWithLog[112];
        snprintf(statusWithLog, sizeof(statusWithLog), "%s log:RAM", statusText);
        return statusWithLog;
    }
    return statusText;
}

#endif
