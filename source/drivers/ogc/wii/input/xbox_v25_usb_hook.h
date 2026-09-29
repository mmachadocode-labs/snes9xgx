#ifndef SNES9XGX_XBOX_V25_USB_HOOK_H
#define SNES9XGX_XBOX_V25_USB_HOOK_H

/*
 * V27 broad IOS58 /dev/usb/ven probe for Xbox One S (045e:02ea).
 *
 * One Wii run now checks the selected handle, SetConfiguration(1), and all
 * matching IOS58 interface entries. The normal driver still owns the real
 * async input path; this shim only records which setup/OUT operations work.
 *
 * q: in Credits is replaced with 27000 + a 16-bit success mask.
 */

#include <ogc/usb.h>
#include <ogc/system.h>
#include <ogc/timesupp.h>
#include <stdbool.h>
#include <string.h>

#define XBOX_V27_VID 0x045e
#define XBOX_V27_PID 0x02ea
#define XBOX_V27_IN_EP  0x82
#define XBOX_V27_OUT_EP 0x02
#define XBOX_V27_MAX_DEVICES 32

/* q = 27000 + mask
 * 0x0001 selected: USB_GetDescriptors succeeded
 * 0x0002 selected: aligned USB_GetConfiguration succeeded
 * 0x0004 selected: USB_SetConfiguration(1) succeeded
 * 0x0008 selected: aligned GetConfiguration after SetConfiguration succeeded
 * 0x0010 selected: power-on OUT after SetConfiguration succeeded
 * 0x0020 selected: One-S init OUT after SetConfiguration succeeded
 * 0x0040 selected: USB_SetAlternativeInterface(0,0) succeeded
 * 0x0080 selected: power-on OUT after SetAlternativeInterface succeeded
 * 0x0100 USB_GetDeviceList succeeded
 * 0x0200 entry 0: USB_OpenDevice succeeded
 * 0x0400 entry 0: power-on OUT succeeded
 * 0x0800 entry 1: USB_OpenDevice succeeded
 * 0x1000 entry 1: power-on OUT succeeded
 * 0x2000 entry 2: USB_OpenDevice succeeded
 * 0x4000 entry 2: power-on OUT succeeded
 * 0x8000 probe MEM2/aligned buffer allocation succeeded
 */
static s32 xboxV27Fd = -1;
static bool xboxV27ProbeRan = false;
static u32 xboxV27Mask = 0;

static inline bool XBOX_V27_Write(s32 fd, const u8 *packet, u16 packetLen,
                                  u8 *buffer)
{
    memset(buffer, 0, 64);
    memcpy(buffer, packet, packetLen);
    s32 rc = USB_WriteIntrMsg(fd, XBOX_V27_OUT_EP, packetLen, buffer);
    udelay(15000);
    return rc >= 0;
}

static inline void XBOX_V27_ProbeEntries(u8 *buffer)
{
    static usb_device_entry entries[XBOX_V27_MAX_DEVICES] ATTRIBUTE_ALIGN(32);
    memset(entries, 0, sizeof(entries));

    u8 count = 0;
    s32 listRc = USB_GetDeviceList(entries, XBOX_V27_MAX_DEVICES, 0xff, &count);
    if (listRc < 0)
        return;

    xboxV27Mask |= 0x0100;

    static const u8 powerOn[] = { 0x05, 0x20, 0x00, 0x01, 0x00 };
    int match = 0;

    for (u8 i = 0; i < count && match < 3; ++i) {
        if (entries[i].vid != XBOX_V27_VID || entries[i].pid != XBOX_V27_PID)
            continue;

        s32 probeFd = -1;
        s32 openRc = USB_OpenDevice(entries[i].device_id, entries[i].vid,
                                    entries[i].pid, &probeFd);
        if (openRc >= 0 && probeFd != -1) {
            if (match == 0) xboxV27Mask |= 0x0200;
            if (match == 1) xboxV27Mask |= 0x0800;
            if (match == 2) xboxV27Mask |= 0x2000;

            if (XBOX_V27_Write(probeFd, powerOn, sizeof(powerOn), buffer)) {
                if (match == 0) xboxV27Mask |= 0x0400;
                if (match == 1) xboxV27Mask |= 0x1000;
                if (match == 2) xboxV27Mask |= 0x4000;
            }
        }
        match++;
    }
}

static inline void XBOX_V27_RunProbe(s32 fd)
{
    if (xboxV27ProbeRan || fd == -1)
        return;

    xboxV27ProbeRan = true;

    static const u8 powerOn[] = { 0x05, 0x20, 0x00, 0x01, 0x00 };
    static const u8 oneSInit[] = { 0x05, 0x20, 0x01, 0x0f, 0x06 };

    /* 128 bytes gives us one 64-byte transfer area and aligned scratch. */
    u8 *block = (u8 *)SYS_AllocArenaMem2Lo(128, 32);
    if (!block)
        return;

    xboxV27Mask |= 0x8000;
    u8 *buffer = block;
    u8 *cfg = block + 64; /* 32-byte aligned */

    /* Re-check the selected interface using correctly aligned storage. */
    usb_devdesc desc;
    memset(&desc, 0, sizeof(desc));
    s32 descRc = USB_GetDescriptors(fd, &desc);
    if (descRc >= 0) {
        xboxV27Mask |= 0x0001;
        USB_FreeDescriptors(&desc);
    }

    memset(cfg, 0, 32);
    if (USB_GetConfiguration(fd, cfg) >= 0)
        xboxV27Mask |= 0x0002;

    /* Test every matching IOS58 entry before changing configuration. */
    XBOX_V27_ProbeEntries(buffer);

    /* A working Wii XInput implementation configures the device before its
     * operational interrupt-OUT command, so test that ordering explicitly. */
    if (USB_SetConfiguration(fd, 1) >= 0)
        xboxV27Mask |= 0x0004;
    udelay(20000);

    memset(cfg, 0, 32);
    if (USB_GetConfiguration(fd, cfg) >= 0)
        xboxV27Mask |= 0x0008;

    if (XBOX_V27_Write(fd, powerOn, sizeof(powerOn), buffer))
        xboxV27Mask |= 0x0010;
    if (XBOX_V27_Write(fd, oneSInit, sizeof(oneSInit), buffer))
        xboxV27Mask |= 0x0020;

    if (USB_SetAlternativeInterface(fd, 0, 0) >= 0)
        xboxV27Mask |= 0x0040;
    udelay(15000);

    if (XBOX_V27_Write(fd, powerOn, sizeof(powerOn), buffer))
        xboxV27Mask |= 0x0080;
}

static inline s32 XBOX_V27_USB_OpenDevice(s32 device_id, u16 vid, u16 pid, s32 *fd)
{
    s32 rc = USB_OpenDevice(device_id, vid, pid, fd);
    if (rc < 0)
        return rc;

    if (vid == XBOX_V27_VID && pid == XBOX_V27_PID && fd && *fd != -1) {
        xboxV27Fd = *fd;
        xboxV27ProbeRan = false;
        xboxV27Mask = 0;
    }

    return rc;
}

static inline s32 XBOX_V27_USB_ReadIntrMsgAsync(s32 fd, u8 endpoint, u16 length,
                                                 void *data, usbcallback cb,
                                                 void *userdata)
{
    /* Preserve xpad ordering: real IN request is queued before the probe. */
    s32 rc = USB_ReadIntrMsgAsync(fd, endpoint, length, data, cb, userdata);

    if (fd == xboxV27Fd && endpoint == XBOX_V27_IN_EP && rc >= 0) {
        XBOX_V27_RunProbe(fd);
        return 27000 + (s32)xboxV27Mask;
    }

    return rc;
}

#define USB_OpenDevice       XBOX_V27_USB_OpenDevice
#define USB_ReadIntrMsgAsync XBOX_V27_USB_ReadIntrMsgAsync

#endif
