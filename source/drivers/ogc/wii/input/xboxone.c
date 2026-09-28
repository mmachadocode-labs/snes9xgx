#ifdef HW_RVL

#include <gccore.h>
#include <ogc/usb.h>
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MS_VID 0x045e
#define GIP_CLASS 0xff
#define MAX_USB_DEVICES 32
#define BUF_SIZE 64
#define STICK_THRESHOLD 16384
#define TRIGGER_THRESHOLD 512
#define USB_NAK -7005

#define GIP_CMD_VIRTUAL_KEY 0x07
#define GIP_CMD_INPUT       0x20
#define GIP_OPT_INTERNAL    0x20

typedef struct {
    u16 pid;
    bool needsSInit;
} xbox_one_device;

static const xbox_one_device supported[] = {
    { 0x02d1, false }, /* Xbox One */
    { 0x02dd, false }, /* Xbox One, 2015 firmware */
    { 0x02e3, false }, /* Xbox One Elite */
    { 0x02ea, true  }, /* Xbox One S */
    { 0x0b00, true  }, /* Elite Series 2 */
    { 0x0b0a, false }, /* Adaptive Controller */
    { 0x0b12, false }, /* Series S|X */
};

static s32 deviceId = 0;
static u8 epIn = 0x82;
static u8 epOut = 0x02;
static u16 packetSize = BUF_SIZE;
static u8 ATTRIBUTE_ALIGN(32) inBuf[BUF_SIZE];
static volatile bool reading = false;
static volatile bool closePending = false;
static u32 held = 0;
static const xbox_one_device *active = NULL;
static char statusText[96] = "not found";
static u8 sequence = 0;

static const xbox_one_device *findDevice(u16 vid, u16 pid)
{
    if (vid != MS_VID)
        return NULL;

    for (unsigned i = 0; i < sizeof(supported) / sizeof(supported[0]); ++i)
        if (supported[i].pid == pid)
            return &supported[i];

    return NULL;
}

static int16_t s16le(const u8 *p)
{
    return (int16_t)((u16)p[0] | ((u16)p[1] << 8));
}

static u16 u16le(const u8 *p)
{
    return (u16)p[0] | ((u16)p[1] << 8);
}

/* libogc's IOS USB path uses IPC/DMA. Buffers passed into USB calls must live
 * in stable 32-byte aligned heap memory; aligned stack locals can still fail
 * or crash on real hardware. */
static int sendPacket(const u8 *packet, u8 len)
{
    if (!packet || len == 0 || len > 32 || deviceId == 0)
        return -1;

    u8 *out = (u8 *)memalign(32, 32);
    if (!out)
        return -1;

    memset(out, 0, 32);
    memcpy(out, packet, len);
    out[2] = sequence;

    int rc = USB_WriteIntrMsg(deviceId, epOut, len, out);
    free(out);

    if (rc >= 0)
        sequence++;
    return rc;
}

static int sendPacketRetry(const u8 *packet, u8 len)
{
    int rc = -1;
    for (int attempt = 0; attempt < 8; ++attempt) {
        rc = sendPacket(packet, len);
        if (rc >= 0)
            return rc;
        if (rc != USB_NAK)
            return rc;
        usleep(20000);
    }
    return rc;
}

static int initializeController(int *secondResult)
{
    static const u8 powerOn[] = {
        0x05, GIP_OPT_INTERNAL, 0x00, 0x01, 0x00
    };
    static const u8 oneSInit[] = {
        0x05, GIP_OPT_INTERNAL, 0x00, 0x0f, 0x06
    };

    sequence = 0;
    int first = sendPacketRetry(powerOn, sizeof(powerOn));
    int second = 0;

    if (first >= 0 && active && active->needsSInit) {
        usleep(20000);
        second = sendPacketRetry(oneSInit, sizeof(oneSInit));
    }

    if (secondResult)
        *secondResult = second;
    return first;
}

static bool findEndpoints(usb_devdesc *desc, u8 *inEp, u8 *outEp, u16 *inSize)
{
    if (!desc || !inEp || !outEp || !inSize)
        return false;

    *inEp = 0;
    *outEp = 0;
    *inSize = 0;

    /* Prefer the Xbox GIP data interface: FF / 47 / D0. */
    for (u8 c = 0; c < desc->bNumConfigurations; ++c) {
        usb_configurationdesc *cfg = &desc->configurations[c];
        for (u8 i = 0; i < cfg->bNumInterfaces; ++i) {
            usb_interfacedesc *itf = &cfg->interfaces[i];
            if (itf->bInterfaceClass != 0xff ||
                itf->bInterfaceSubClass != 0x47 ||
                itf->bInterfaceProtocol != 0xd0)
                continue;

            for (u8 e = 0; e < itf->bNumEndpoints; ++e) {
                usb_endpointdesc *ep = &itf->endpoints[e];
                if ((ep->bmAttributes & 0x03) != USB_ENDPOINT_INTERRUPT)
                    continue;

                if ((ep->bEndpointAddress & USB_ENDPOINT_IN) && !*inEp) {
                    *inEp = ep->bEndpointAddress;
                    *inSize = ep->wMaxPacketSize;
                } else if (!(ep->bEndpointAddress & USB_ENDPOINT_IN) && !*outEp) {
                    *outEp = ep->bEndpointAddress;
                }
            }

            if (*inEp && *outEp)
                return true;
        }
    }

    /* Fallback: first interrupt IN/OUT pair, same strategy used by the Wii
     * USB gamepad code in ioQuake3-wii. */
    for (u8 c = 0; c < desc->bNumConfigurations; ++c) {
        usb_configurationdesc *cfg = &desc->configurations[c];
        for (u8 i = 0; i < cfg->bNumInterfaces; ++i) {
            usb_interfacedesc *itf = &cfg->interfaces[i];
            for (u8 e = 0; e < itf->bNumEndpoints; ++e) {
                usb_endpointdesc *ep = &itf->endpoints[e];
                if ((ep->bmAttributes & 0x03) != USB_ENDPOINT_INTERRUPT)
                    continue;

                if ((ep->bEndpointAddress & USB_ENDPOINT_IN) && !*inEp) {
                    *inEp = ep->bEndpointAddress;
                    *inSize = ep->wMaxPacketSize;
                } else if (!(ep->bEndpointAddress & USB_ENDPOINT_IN) && !*outEp) {
                    *outEp = ep->bEndpointAddress;
                }
            }
        }
    }

    return *inEp && *outEp;
}

static void parseInput(int len)
{
    if (len < 5)
        return;

    if (inBuf[0] == GIP_CMD_VIRTUAL_KEY) {
        snprintf(statusText, sizeof(statusText), "rx:%d cmd:07", len);
        return;
    }

    if (inBuf[0] != GIP_CMD_INPUT || len < 18) {
        snprintf(statusText, sizeof(statusText), "rx:%d cmd:%02x", len, inBuf[0]);
        return;
    }

    held = 0;

    held |= (inBuf[4] & 0x04) ? PAD_BUTTON_START : 0;
    held |= (inBuf[4] & 0x08) ? PAD_TRIGGER_Z : 0;
    held |= (inBuf[4] & 0x10) ? PAD_BUTTON_B : 0;
    held |= (inBuf[4] & 0x20) ? PAD_BUTTON_A : 0;
    held |= (inBuf[4] & 0x40) ? PAD_BUTTON_Y : 0;
    held |= (inBuf[4] & 0x80) ? PAD_BUTTON_X : 0;

    held |= (inBuf[5] & 0x01) ? PAD_BUTTON_UP : 0;
    held |= (inBuf[5] & 0x02) ? PAD_BUTTON_DOWN : 0;
    held |= (inBuf[5] & 0x04) ? PAD_BUTTON_LEFT : 0;
    held |= (inBuf[5] & 0x08) ? PAD_BUTTON_RIGHT : 0;
    held |= (inBuf[5] & 0x10) ? PAD_TRIGGER_L : 0;
    held |= (inBuf[5] & 0x20) ? PAD_TRIGGER_R : 0;

    if (u16le(&inBuf[6]) > TRIGGER_THRESHOLD)
        held |= PAD_TRIGGER_L;
    if (u16le(&inBuf[8]) > TRIGGER_THRESHOLD)
        held |= PAD_TRIGGER_R;

    int16_t lx = s16le(&inBuf[10]);
    int16_t ly = s16le(&inBuf[12]);
    int16_t rx = s16le(&inBuf[14]);
    int16_t ry = s16le(&inBuf[16]);

    held |= (ly > STICK_THRESHOLD) ? PAD_BUTTON_UP : 0;
    held |= (ly < -STICK_THRESHOLD) ? PAD_BUTTON_DOWN : 0;
    held |= (lx < -STICK_THRESHOLD) ? PAD_BUTTON_LEFT : 0;
    held |= (lx > STICK_THRESHOLD) ? PAD_BUTTON_RIGHT : 0;
    held |= (ry > STICK_THRESHOLD) ? PAD_BUTTON_UP : 0;
    held |= (ry < -STICK_THRESHOLD) ? PAD_BUTTON_DOWN : 0;
    held |= (rx < -STICK_THRESHOLD) ? PAD_BUTTON_LEFT : 0;
    held |= (rx > STICK_THRESHOLD) ? PAD_BUTTON_RIGHT : 0;

    snprintf(statusText, sizeof(statusText), "INPUT len:%d held:%08lx", len, (unsigned long)held);
}

static int readCallback(int result, void *userdata)
{
    (void)userdata;

    if (!reading || deviceId == 0)
        return 1;

    if (result <= 0) {
        reading = false;
        closePending = true;
        held = 0;
        snprintf(statusText, sizeof(statusText), "read:%d", result);
        return 1;
    }

    parseInput(result);

    if (!reading || closePending || deviceId == 0)
        return 1;

    int rc = USB_ReadIntrMsgAsync(deviceId, epIn, packetSize, inBuf,
                                  &readCallback, NULL);
    if (rc < 0) {
        reading = false;
        closePending = true;
        snprintf(statusText, sizeof(statusText), "requeue:%d", rc);
    }

    return 1;
}

static int removalCallback(int result, void *userdata)
{
    (void)result;
    (void)userdata;

    reading = false;
    closePending = true;
    held = 0;
    strcpy(statusText, "removed");
    return 1;
}

static void cleanupClosedController(void)
{
    if (!closePending || deviceId == 0)
        return;

    s32 fd = deviceId;
    reading = false;
    deviceId = 0;
    active = NULL;
    held = 0;
    closePending = false;
    USB_CloseDevice(&fd);
}

static void openController(void)
{
    cleanupClosedController();
    if (deviceId != 0)
        return;

    usb_device_entry *devices = (usb_device_entry *)memalign(
        32, sizeof(usb_device_entry) * MAX_USB_DEVICES);
    if (!devices) {
        strcpy(statusText, "alloc devlist failed");
        return;
    }
    memset(devices, 0, sizeof(usb_device_entry) * MAX_USB_DEVICES);

    u8 count = 0;
    s32 listResult = USB_GetDeviceList(devices, MAX_USB_DEVICES, 0, &count);
    if (listResult < 0) {
        count = 0;
        listResult = USB_GetDeviceList(devices, MAX_USB_DEVICES, GIP_CLASS, &count);
    }

    if (listResult < 0) {
        snprintf(statusText, sizeof(statusText), "list:%d", listResult);
        free(devices);
        return;
    }

    bool sawSupported = false;
    char lastDiag[96] = "Xbox found";

    for (u8 i = 0; i < count; ++i) {
        const xbox_one_device *candidate = findDevice(devices[i].vid, devices[i].pid);
        if (!candidate)
            continue;

        sawSupported = true;

        s32 fd = -1;
        s32 openResult = USB_OpenDevice(devices[i].device_id,
                                        devices[i].vid,
                                        devices[i].pid,
                                        &fd);
        if (openResult < 0) {
            snprintf(lastDiag, sizeof(lastDiag), "i%u open:%d", i, openResult);
            continue;
        }

        /* This top-level descriptor must also be heap-backed and 32-byte
         * aligned. A stack usb_devdesc was the source of our previous d:-4. */
        usb_devdesc *desc = (usb_devdesc *)memalign(32, sizeof(usb_devdesc));
        s32 descResult = -1;
        bool endpointsFound = false;
        u8 foundIn = 0;
        u8 foundOut = 0;
        u16 foundSize = 0;

        if (desc) {
            memset(desc, 0, sizeof(*desc));
            descResult = USB_GetDescriptors(fd, desc);
            if (descResult >= 0) {
                endpointsFound = findEndpoints(desc, &foundIn, &foundOut, &foundSize);
                USB_FreeDescriptors(desc);
            }
            free(desc);
        }

        /* Known values for 045e:02ea if descriptor parsing still fails. */
        epIn = endpointsFound ? foundIn : 0x82;
        epOut = endpointsFound ? foundOut : 0x02;
        packetSize = (endpointsFound && foundSize > 0 && foundSize <= BUF_SIZE)
                         ? foundSize
                         : BUF_SIZE;

        deviceId = fd;
        active = candidate;
        held = 0;
        closePending = false;

        /* Follow the existing Wii USB-gamepad implementation: initialize the
         * pad first, then arm the interrupt IN transfer. */
        int secondWrite = 0;
        int firstWrite = initializeController(&secondWrite);

        reading = true;
        int queueResult = USB_ReadIntrMsgAsync(deviceId, epIn, packetSize, inBuf,
                                               &readCallback, NULL);
        if (queueResult < 0)
            reading = false;

        if (queueResult >= 0) {
            USB_DeviceRemovalNotifyAsync(fd, &removalCallback, NULL);
            snprintf(statusText, sizeof(statusText),
                     "V10 i%u d:%d ep:%02x/%02x w:%d/%d q:%d",
                     i, descResult, epIn, epOut,
                     firstWrite, secondWrite, queueResult);
            free(devices);
            return;
        }

        snprintf(lastDiag, sizeof(lastDiag),
                 "i%u d:%d ep:%02x/%02x w:%d/%d q:%d",
                 i, descResult, epIn, epOut,
                 firstWrite, secondWrite, queueResult);

        deviceId = 0;
        active = NULL;
        reading = false;
        USB_CloseDevice(&fd);
    }

    if (sawSupported)
        snprintf(statusText, sizeof(statusText), "%s", lastDiag);
    else
        snprintf(statusText, sizeof(statusText), "not found (%u USB)", count);

    free(devices);
}

void XBOXONE_ScanPads(void)
{
    cleanupClosedController();
    if (deviceId == 0)
        openController();
}

u32 XBOXONE_ButtonsHeld(int chan)
{
    cleanupClosedController();
    if (deviceId == 0)
        openController();

    if (chan != 0 || deviceId == 0)
        return 0;

    return held;
}

char *XBOXONE_Status(void)
{
    cleanupClosedController();
    if (deviceId == 0)
        openController();

    return statusText;
}

#endif
