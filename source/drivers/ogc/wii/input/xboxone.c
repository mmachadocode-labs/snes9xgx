#ifdef HW_RVL

#include <gccore.h>
#include <ogc/usb.h>
#include <stdio.h>
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

static int sendPacket(const u8 *packet, u8 len)
{
    u8 ATTRIBUTE_ALIGN(32) out[32];
    if (!packet || len == 0 || len > sizeof(out) || deviceId == 0)
        return -1;

    memset(out, 0, sizeof(out));
    memcpy(out, packet, len);
    out[2] = sequence;

    int rc = USB_WriteIntrMsg(deviceId, epOut, len, out);
    if (rc >= 0)
        sequence++;
    return rc;
}

static int sendPacketRetry(const u8 *packet, u8 len)
{
    int rc = -1;
    for (int attempt = 0; attempt < 40; ++attempt) {
        rc = sendPacket(packet, len);
        if (rc >= 0)
            return rc;
        if (rc != USB_NAK)
            return rc;
        usleep(25000);
    }
    return rc;
}

/* Linux xpad order for Xbox One S: normal power-on first, then the
 * model-specific packet used by 045e:02ea. IOS58 may expose NAK while the
 * device is waking, so retry the same packet/sequence rather than advancing. */
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
        usleep(25000);
        second = sendPacketRetry(oneSInit, sizeof(oneSInit));
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
    if (!reading || deviceId == 0)
        return 1;

    if (result > 0)
        parseInput(result);
    else if (result < 0)
        snprintf(statusText, sizeof(statusText), "read cb:%d", result);

    if (result == USB_NAK)
        usleep(5000);

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

        /* USB_OpenDevice already requests resume, but IOS58 does not wait for
         * the physical pad to become ready. Explicitly resume and give the
         * Xbox One S time to leave suspend before touching endpoint 0x02. */
        s32 resumeResult = USB_ResumeDevice(fd);
        usleep(250000);

        usb_devdesc desc;
        s32 descResult = USB_GetDescriptors(fd, &desc);
        if (descResult >= 0)
            USB_FreeDescriptors(&desc);

        /* 045e:02ea interface 0 is FF/47/D0 with interrupt OUT 0x02 and
         * interrupt IN 0x82, 64 bytes. Do not SET_CONFIGURATION/INTERFACE:
         * IOS58 already owns/configures the VEN interface and those requests
         * were returning NAK on the real Wii. */
        epIn = 0x82;
        epOut = 0x02;
        packetSize = BUF_SIZE;

        deviceId = fd;
        active = candidate;
        held = 0;
        reading = true;

        /* Arm input first. Linux keeps an input URB active while the startup
         * packets are sent; doing the same also tells us whether IN works even
         * when the first OUT transfer is NAKed. */
        int queueResult = USB_ReadIntrMsgAsync(deviceId, epIn, packetSize, inBuf, &readCallback, NULL);
        if (queueResult < 0)
            reading = false;

        int secondWrite = 0;
        int firstWrite = initializeController(&secondWrite);

        if (queueResult >= 0) {
            USB_DeviceRemovalNotifyAsync(fd, &removalCallback, (void *)fd);
            snprintf(statusText, sizeof(statusText),
                     "OPEN i%u r:%d d:%d q:%d w:%d/%d",
                     i, resumeResult, descResult, queueResult, firstWrite, secondWrite);
            return;
        }

        snprintf(lastDiag, sizeof(lastDiag),
                 "i%u r:%d d:%d q:%d w:%d/%d",
                 i, resumeResult, descResult, queueResult, firstWrite, secondWrite);
        deviceId = 0;
        active = NULL;
        USB_CloseDevice(&fd);
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
