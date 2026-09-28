#ifdef HW_RVL

/*
 * Wired Xbox One input for Snes9x GX.
 *
 * USB transport/lifetime handling is adapted from Mayo1970/ioQuake3-wii's
 * code/input/wii_usb_hid.c (GPLv2).  In particular: IOS IPC buffers live on
 * a 32-byte-aligned heap, the controller is opened through its IOS58 V5
 * device_id, configuration is left to IOS, hotplug is polled, and an async
 * read callback never closes/reopens a USB device itself.
 */

#include <gccore.h>
#include <ogc/usb.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>

#define MICROSOFT_VID 0x045e
#define USB_CLASS_VENDOR_SPECIFIC 0xff
#define MAX_USB_DEVICES 16
#define MAX_REPORT_SIZE 64
#define POLL_INTERVAL_FRAMES 60
#define STICK_THRESHOLD 16384
#define TRIGGER_THRESHOLD 128

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
    { 0x02ea, "Xbox One S" },
    { 0x0b12, "Xbox Elite 2" },
    { 0x0b13, "Xbox Series X/S" },
};

static bool initialized = false;
static bool active = false;
static volatile bool closePending = false;
static volatile s32 asyncError = 0;
static volatile s32 deviceFd = -1;
static const xbox_profile *activeProfile = NULL;
static u8 endpointIn = 0;
static u8 endpointOut = 0;
static u16 reportLength = MAX_REPORT_SIZE;
static volatile u32 heldButtons = 0;
static volatile bool guidePressed = false;
static int pollCountdown = 0;
static char statusText[96] = "not found";

/* IOS DMAs directly into this buffer. Keep it static and 32-byte aligned. */
static u8 ATTRIBUTE_ALIGN(32) reportBuffer[MAX_REPORT_SIZE];

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

    /* Xbox/Guide is delivered as a separate GIP virtual-key packet. */
    if (d[0] == GIP_CMD_VIRTUAL_KEY) {
        guidePressed = (d[4] & 0x01) != 0;
        return;
    }

    if (d[0] != GIP_CMD_INPUT || len < 18)
        return;

    u32 buttons = 0;

    /* GIP layout follows Linux xpad's xpadone_process_packet(). */
    if (d[4] & 0x04) buttons |= PAD_BUTTON_START; /* Menu */
    if (d[4] & 0x08) buttons |= PAD_TRIGGER_Z;    /* View */

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

    /* Xbox One triggers are 10-bit little-endian values. */
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

    /* Do not log, close or reopen USB from an IOS callback. */
    if (result < 0) {
        asyncError = result;
        closePending = true;
        return 0;
    }

    if (result > 0)
        parseXboxOneReport(reportBuffer, (u16)result);

    if (deviceFd >= 0) {
        s32 rc = USB_ReadIntrMsgAsync(deviceFd, endpointIn, reportLength,
                                      reportBuffer, readCallback, NULL);
        if (rc < 0) {
            asyncError = rc;
            closePending = true;
        }
    }
    return 0;
}

static void closeController(void)
{
    s32 fd = deviceFd;
    deviceFd = -1; /* callback sees invalid fd before the blocking close */

    if (fd >= 0)
        USB_CloseDevice(&fd);

    active = false;
    activeProfile = NULL;
    endpointIn = endpointOut = 0;
    heldButtons = 0;
    guidePressed = false;
    closePending = false;
}

/* Prefer the Xbox GIP interface FF/47/D0, then fall back to the first pair of
 * interrupt endpoints exactly like the generic ioQuake3-wii implementation. */
static bool findEndpoints(usb_devdesc *dd, u8 *inEp, u8 *outEp)
{
    if (!dd || !inEp || !outEp)
        return false;

    *inEp = 0;
    *outEp = 0;

    /* First pass: GIP data interface. */
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

    /* Fallback: first interrupt IN/OUT pair. */
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

static s32 initializeXboxOne(s32 fd, u8 outEp)
{
    static const u8 powerOn[] = { 0x05, 0x20, 0x00, 0x01, 0x00 };

    if (!outEp)
        return -1;

    /* Proven ioQuake3-wii pattern: heap/memalign, never a stack IPC buffer. */
    u8 *out = (u8 *)memalign(32, 32);
    if (!out)
        return -1;

    memset(out, 0, 32);
    memcpy(out, powerOn, sizeof(powerOn));
    s32 rc = USB_WriteIntrMsg(fd, outEp, sizeof(powerOn), out);
    free(out);
    return rc;
}

static bool tryOpen(const usb_device_entry *entry)
{
    const xbox_profile *profile = findProfile(entry->vid, entry->pid);
    if (!profile)
        return false;

    /* IOS IPC descriptor storage must also be heap-aligned. */
    usb_devdesc *dd = (usb_devdesc *)memalign(32, sizeof(usb_devdesc));
    if (!dd) {
        snprintf(statusText, sizeof(statusText), "alloc desc failed");
        return false;
    }
    memset(dd, 0, sizeof(*dd));

    s32 fd = -1;
    s32 rc = USB_OpenDevice(entry->device_id, entry->vid, entry->pid, &fd);
    if (rc < 0) {
        snprintf(statusText, sizeof(statusText), "open:%d %04x:%04x",
                 rc, entry->vid, entry->pid);
        free(dd);
        return false;
    }

    rc = USB_GetDescriptors(fd, dd);
    if (rc < 0) {
        snprintf(statusText, sizeof(statusText), "desc:%d %04x:%04x",
                 rc, entry->vid, entry->pid);
        USB_CloseDevice(&fd);
        free(dd);
        return false;
    }

    u8 inEp = 0, outEp = 0;
    bool endpointsOk = findEndpoints(dd, &inEp, &outEp);
    USB_FreeDescriptors(dd);
    free(dd);

    if (!endpointsOk || !inEp || !outEp) {
        snprintf(statusText, sizeof(statusText), "no endpoints %04x:%04x",
                 entry->vid, entry->pid);
        USB_CloseDevice(&fd);
        return false;
    }

    /* Do NOT call USB_SetConfiguration/USB_SetAlternativeInterface here.
     * IOS58 already configured the real device; ioQuake3-wii found that Xbox
     * One and DS4 reject those calls. */
    rc = initializeXboxOne(fd, outEp);
    if (rc < 0) {
        snprintf(statusText, sizeof(statusText), "init:%d ep:%02x/%02x",
                 rc, inEp, outEp);
        USB_CloseDevice(&fd);
        return false;
    }

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

    rc = USB_ReadIntrMsgAsync(deviceFd, endpointIn, reportLength,
                              reportBuffer, readCallback, NULL);
    if (rc < 0) {
        snprintf(statusText, sizeof(statusText), "readq:%d ep:%02x/%02x",
                 rc, endpointIn, endpointOut);
        closeController();
        return false;
    }

    snprintf(statusText, sizeof(statusText), "connected %s ep:%02x/%02x",
             profile->name, endpointIn, endpointOut);
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

    bool sawSupported = false;
    for (u8 i = 0; i < count; ++i) {
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

    initialized = true;
    active = false;
    deviceFd = -1;
    heldButtons = 0;
    closePending = false;
    asyncError = 0;

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
        snprintf(statusText, sizeof(statusText), "disconnected read:%d", err);
        pollCountdown = 0;
    }

    if (active)
        return;

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
    return statusText;
}

#endif
