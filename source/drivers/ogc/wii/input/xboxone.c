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

#define GIP_CMD_POWER        0x05
#define GIP_CMD_AUTHENTICATE 0x06
#define GIP_CMD_VIRTUAL_KEY  0x07
#define GIP_CMD_LED          0x0a
#define GIP_CMD_INPUT        0x20
#define GIP_OPT_INTERNAL     0x20

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
static char statusText[64] = "not found";
static u8 sequence = 0;
static int lastReadResult = 0;

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

static void initializeController(void)
{
    static const u8 powerOn[] = {
        GIP_CMD_POWER, GIP_OPT_INTERNAL, 0x00, 0x01, 0x00
    };
    static const u8 oneSInit[] = {
        GIP_CMD_POWER, GIP_OPT_INTERNAL, 0x00, 0x0f, 0x06
    };
    static const u8 ledOn[] = {
        GIP_CMD_LED, GIP_OPT_INTERNAL, 0x00, 0x03, 0x00, 0x01, 0x14
    };
    static const u8 authDone[] = {
        GIP_CMD_AUTHENTICATE, GIP_OPT_INTERNAL, 0x00, 0x02, 0x01, 0x00
    };

    sequence = 0;
    sendPacket(powerOn, sizeof(powerOn));
    usleep(2000);

    if (active && active->needsSInit) {
        sendPacket(oneSInit, sizeof(oneSInit));
        usleep(2000);
    }

    sendPacket(ledOn, sizeof(ledOn));
    usleep(2000);
    sendPacket(authDone, sizeof(authDone));
}

static void parseInput(int len)
{
    if (len < 5)
        return;

    if (inBuf[0] == GIP_CMD_VIRTUAL_KEY)
        return;

    if (inBuf[0] != GIP_CMD_INPUT || len < 18)
        return;

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

    lastReadResult = result;

    if (result > 0)
        parseInput(result);
    else if (result < 0)
        snprintf(statusText, sizeof(statusText), "connected readerr:%d", result);

    int rc = USB_ReadIntrMsgAsync(deviceId, epIn, packetSize, inBuf, &readCallback, NULL);
    if (rc < 0) {
        reading = false;
        snprintf(statusText, sizeof(statusText), "connected queueerr:%d", rc);
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
        snprintf(statusText, sizeof(statusText), "list err:%d", listResult);
        return;
    }

    for (u8 i = 0; i < count; ++i) {
        const xbox_one_device *candidate = findDevice(devices[i].vid, devices[i].pid);
        if (!candidate)
            continue;

        snprintf(statusText, sizeof(statusText), "found %04x:%04x", devices[i].vid, devices[i].pid);

        s32 fd = -1;
        s32 openResult = USB_OpenDevice(devices[i].device_id, devices[i].vid, devices[i].pid, &fd);
        if (openResult < 0) {
            snprintf(statusText, sizeof(statusText), "open err:%d", openResult);
            continue;
        }

        usb_devdesc desc;
        s32 descResult = USB_GetDescriptors(fd, &desc);
        if (descResult < 0) {
            snprintf(statusText, sizeof(statusText), "desc err:%d", descResult);
            USB_CloseDevice(&fd);
            continue;
        }

        u8 config = 1;
        if (!findDataEndpoints(&desc, &config)) {
            strcpy(statusText, "endpoint not found");
            USB_FreeDescriptors(&desc);
            USB_CloseDevice(&fd);
            continue;
        }

        /* IOS58 often already owns/configures V5 devices.  Try SET_CONFIGURATION,
           but do not reject a valid opened device solely because it is already set. */
        USB_SetConfiguration(fd, config);

        deviceId = fd;
        active = candidate;
        held = 0;
        reading = false;
        lastReadResult = 0;

        snprintf(statusText, sizeof(statusText), "connected wait ep:%02x/%02x", epIn, epOut);
        initializeController();
        USB_DeviceRemovalNotifyAsync(fd, &removalCallback, (void *)fd);
        USB_FreeDescriptors(&desc);
        return;
    }

    snprintf(statusText, sizeof(statusText), "not found (%u USB)", count);
}

void XBOXONE_ScanPads(void)
{
    if (deviceId == 0)
        return;

    if (!reading) {
        reading = true;
        int rc = USB_ReadIntrMsgAsync(deviceId, epIn, packetSize, inBuf, &readCallback, NULL);
        if (rc < 0) {
            reading = false;
            snprintf(statusText, sizeof(statusText), "connected queueerr:%d", rc);
        }
    }
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
