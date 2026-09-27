#ifdef HW_RVL

#include <gccore.h>
#include <ogc/usb.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define MS_VID 0x045e
#define GIP_CLASS 0xff
#define GIP_SUBCLASS 0x47
#define GIP_PROTOCOL 0xd0
#define MAX_USB_DEVICES 32
#define BUF_SIZE 64
#define STICK_THRESHOLD 16384
#define TRIGGER_THRESHOLD 512

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
static u8 epIn = 0x81;
static u8 epOut = 0x01;
static u16 packetSize = BUF_SIZE;
static u8 ATTRIBUTE_ALIGN(32) inBuf[BUF_SIZE];
static bool reading = false;
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

static bool findDataEndpoints(usb_devdesc *desc, u8 *configuration)
{
    bool fallbackFound = false;
    u8 fallbackIn = 0;
    u8 fallbackOut = 0;
    u16 fallbackSize = BUF_SIZE;
    u8 fallbackConfig = 1;

    if (!desc || !desc->configurations)
        return false;

    for (u8 c = 0; c < desc->bNumConfigurations; ++c) {
        usb_configurationdesc *cfg = &desc->configurations[c];
        if (!cfg->interfaces)
            continue;

        for (u8 i = 0; i < cfg->bNumInterfaces; ++i) {
            usb_interfacedesc *intf = &cfg->interfaces[i];
            if (!intf->endpoints)
                continue;

            u8 localIn = 0;
            u8 localOut = 0;
            u16 localSize = BUF_SIZE;

            for (u8 e = 0; e < intf->bNumEndpoints; ++e) {
                usb_endpointdesc *ep = &intf->endpoints[e];
                if ((ep->bmAttributes & 0x03) != USB_ENDPOINT_INTERRUPT)
                    continue;

                if ((ep->bEndpointAddress & 0x80) == USB_ENDPOINT_IN) {
                    localIn = ep->bEndpointAddress;
                    if (ep->wMaxPacketSize > 0 && ep->wMaxPacketSize <= BUF_SIZE)
                        localSize = ep->wMaxPacketSize;
                } else {
                    localOut = ep->bEndpointAddress;
                }
            }

            if (localIn && localOut) {
                if (!fallbackFound) {
                    fallbackFound = true;
                    fallbackIn = localIn;
                    fallbackOut = localOut;
                    fallbackSize = localSize;
                    fallbackConfig = cfg->bConfigurationValue;
                }

                if (intf->bInterfaceClass == GIP_CLASS &&
                    intf->bInterfaceSubClass == GIP_SUBCLASS &&
                    intf->bInterfaceProtocol == GIP_PROTOCOL) {
                    epIn = localIn;
                    epOut = localOut;
                    packetSize = localSize;
                    *configuration = cfg->bConfigurationValue;
                    return true;
                }
            }
        }
    }

    if (fallbackFound) {
        epIn = fallbackIn;
        epOut = fallbackOut;
        packetSize = fallbackSize;
        *configuration = fallbackConfig;
        return true;
    }

    return false;
}

static int sendPacket(const u8 *packet, u8 len)
{
    u8 ATTRIBUTE_ALIGN(32) out[32];
    if (!packet || len == 0 || len > sizeof(out) || deviceId == 0)
        return -1;

    memset(out, 0, sizeof(out));
    memcpy(out, packet, len);
    out[2] = sequence++;
    return USB_WriteIntrMsg(deviceId, epOut, len, out);
}

/* Match the Linux xpad startup sequence: generic 2015+ power-on, then the
 * extra Xbox One S packet for 045e:02ea / Elite 2. */
static int initializeController(int *secondResult)
{
    static const u8 powerOn[] = {
        0x05, GIP_OPT_INTERNAL, 0x00, 0x01, 0x00
    };
    static const u8 oneSInit[] = {
        0x05, GIP_OPT_INTERNAL, 0x00, 0x0f, 0x06
    };

    sequence = 0;
    int first = sendPacket(powerOn, sizeof(powerOn));
    int second = 0;
    usleep(2000);

    if (first >= 0 && active && active->needsSInit) {
        second = sendPacket(oneSInit, sizeof(oneSInit));
        usleep(2000);
    }

    if (secondResult)
        *secondResult = second;
    return first;
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
    held |= (inBuf[4] & 0x10) ? PAD_BUTTON_B : 0; /* Xbox A -> SNES B */
    held |= (inBuf[4] & 0x20) ? PAD_BUTTON_A : 0; /* Xbox B -> SNES A */
    held |= (inBuf[4] & 0x40) ? PAD_BUTTON_Y : 0; /* Xbox X -> SNES Y */
    held |= (inBuf[4] & 0x80) ? PAD_BUTTON_X : 0; /* Xbox Y -> SNES X */

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

    snprintf(statusText, sizeof(statusText), "connected input:%d", len);
}

static int readCallback(int result, void *userdata)
{
    if (!reading || deviceId == 0)
        return 1;

    if (result > 0)
        parseInput(result);
    else if (result < 0)
        snprintf(statusText, sizeof(statusText), "read cb:%d", result);

    int rc = USB_ReadIntrMsgAsync(deviceId, epIn, packetSize, inBuf, &readCallback, NULL);
    if (rc < 0) {
        reading = false;
        snprintf(statusText, sizeof(statusText), "requeue:%d", rc);
    }

    return 1;
}

static int removalCallback(int result, void *userdata)
{
    s32 fd = (s32)userdata;
    if (fd == deviceId) {
        reading = false;
        deviceId = 0;
        active = NULL;
        held = 0;
        strcpy(statusText, "removed");
    }
    return 1;
}

static void openController(void)
{
    if (deviceId != 0)
        return;

    usb_device_entry devices[MAX_USB_DEVICES];
    memset(devices, 0, sizeof(devices));

    u8 count = 0;
    s32 listResult = USB_GetDeviceList(devices, MAX_USB_DEVICES, 0, &count);
    if (listResult < 0) {
        count = 0;
        listResult = USB_GetDeviceList(devices, MAX_USB_DEVICES, GIP_CLASS, &count);
    }

    if (listResult < 0) {
        snprintf(statusText, sizeof(statusText), "list:%d", listResult);
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
        s32 openResult = USB_OpenDevice(devices[i].device_id, devices[i].vid, devices[i].pid, &fd);
        if (openResult < 0) {
            snprintf(lastDiag, sizeof(lastDiag), "i%u id:%d open:%d", i, devices[i].device_id, openResult);
            continue;
        }

        bool descriptorOk = false;
        usb_devdesc desc;
        s32 descResult = USB_GetDescriptors(fd, &desc);
        u8 config = 1;

        if (descResult >= 0) {
            descriptorOk = findDataEndpoints(&desc, &config);
            USB_FreeDescriptors(&desc);
        }

        if (!descriptorOk) {
            epIn = 0x81;
            epOut = 0x01;
            packetSize = BUF_SIZE;
            config = 1;
        }

        /* Explicitly request the standard wired GIP configuration/interface.
         * Keep going even if IOS58 reports that it is already configured. */
        s32 configResult = USB_SetConfiguration(fd, config);
        s32 altResult = USB_SetAlternativeInterface(fd, 0, 0);

        deviceId = fd;
        active = candidate;
        held = 0;
        reading = false;

        int secondWrite = 0;
        int firstWrite = initializeController(&secondWrite);
        if (firstWrite < 0 || secondWrite < 0) {
            snprintf(lastDiag, sizeof(lastDiag),
                     "i%u id:%d d:%d c:%d a:%d w:%d/%d",
                     i, devices[i].device_id, descResult,
                     configResult, altResult, firstWrite, secondWrite);
            deviceId = 0;
            active = NULL;
            USB_CloseDevice(&fd);
            continue;
        }

        reading = true;
        int queueResult = USB_ReadIntrMsgAsync(deviceId, epIn, packetSize, inBuf, &readCallback, NULL);
        if (queueResult < 0) {
            snprintf(lastDiag, sizeof(lastDiag),
                     "i%u id:%d d:%d c:%d a:%d w:%d/%d q:%d",
                     i, devices[i].device_id, descResult,
                     configResult, altResult, firstWrite, secondWrite, queueResult);
            reading = false;
            deviceId = 0;
            active = NULL;
            USB_CloseDevice(&fd);
            continue;
        }

        USB_DeviceRemovalNotifyAsync(fd, &removalCallback, (void *)fd);
        snprintf(statusText, sizeof(statusText),
                 "OK i%u id:%d d:%d c:%d a:%d w:%d/%d q:%d",
                 i, devices[i].device_id, descResult,
                 configResult, altResult, firstWrite, secondWrite, queueResult);
        return;
    }

    if (sawSupported)
        snprintf(statusText, sizeof(statusText), "%s", lastDiag);
    else
        snprintf(statusText, sizeof(statusText), "not found (%u USB)", count);
}

void XBOXONE_ScanPads(void)
{
    if (deviceId == 0)
        openController();
}

u32 XBOXONE_ButtonsHeld(int chan)
{
    if (deviceId == 0)
        openController();

    if (chan != 0 || deviceId == 0)
        return 0;

    return held;
}

char *XBOXONE_Status(void)
{
    if (deviceId == 0)
        openController();

    return statusText;
}

#endif
