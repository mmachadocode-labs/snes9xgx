#ifndef SNES9XGX_XBOX_V25_USB_HOOK_H
#define SNES9XGX_XBOX_V25_USB_HOOK_H

/*
 * V26 comprehensive IOS58 /dev/usb/ven probe for Xbox One S (045e:02ea).
 *
 * Goal: gather several transport answers in one Wii run instead of producing
 * one build per hypothesis. The normal driver still owns input parsing and its
 * async xpad-style init. This shim only probes setup/OUT variants once, after
 * the driver's first interrupt-IN request has been successfully queued.
 *
 * q: in Credits is replaced with 26000 + a bit mask while the real IN request
 * is still queued normally. Bit meanings are documented below.
 */

#include <ogc/usb.h>
#include <ogc/system.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#define XBOX_V26_VID 0x045e
#define XBOX_V26_PID 0x02ea
#define XBOX_V26_IN_EP  0x82
#define XBOX_V26_OUT_EP 0x02

/* q = 26000 + mask
 * 0x001 USB_GetDescriptors succeeded
 * 0x002 USB_GetConfiguration succeeded
 * 0x004 power-on, 5-byte interrupt OUT succeeded
 * 0x008 power-on, 64-byte padded interrupt OUT succeeded
 * 0x010 One-S init seq=0, 5-byte interrupt OUT succeeded
 * 0x020 One-S init seq=1, 5-byte interrupt OUT succeeded
 * 0x040 One-S init seq=1, 64-byte padded interrupt OUT succeeded
 * 0x080 USB_ClearHalt(OUT 0x02) succeeded (V5: cancel endpoint)
 * 0x100 power-on after ClearHalt succeeded
 * 0x200 USB_SetAlternativeInterface(0,0) succeeded
 * 0x400 power-on after SetAlternativeInterface succeeded
 * 0x800 MEM2 probe buffer allocation succeeded
 */
static s32 xboxV26Fd = -1;
static bool xboxV26ProbeRan = false;
static u32 xboxV26Mask = 0;

static inline void XBOX_V26_WriteProbe(s32 fd, const u8 *packet, u16 packetLen,
                                       u16 transferLen, u32 successBit,
                                       u8 *buffer)
{
    memset(buffer, 0, 64);
    memcpy(buffer, packet, packetLen);
    s32 rc = USB_WriteIntrMsg(fd, XBOX_V26_OUT_EP, transferLen, buffer);
    if (rc >= 0)
        xboxV26Mask |= successBit;
    usleep(15000);
}

static inline void XBOX_V26_RunProbe(s32 fd)
{
    if (xboxV26ProbeRan || fd == -1)
        return;

    xboxV26ProbeRan = true;

    static const u8 powerOn[]  = { 0x05, 0x20, 0x00, 0x01, 0x00 };
    static const u8 oneSSeq0[] = { 0x05, 0x20, 0x00, 0x0f, 0x06 };
    static const u8 oneSSeq1[] = { 0x05, 0x20, 0x01, 0x0f, 0x06 };

    u8 *buffer = (u8 *)SYS_AllocArenaMem2Lo(64, 32);
    if (!buffer)
        return;

    xboxV26Mask |= 0x800;

    /* Baseline transfer-length and GIP-sequence matrix. */
    XBOX_V26_WriteProbe(fd, powerOn,  sizeof(powerOn),  5,  0x004, buffer);
    XBOX_V26_WriteProbe(fd, powerOn,  sizeof(powerOn),  64, 0x008, buffer);
    XBOX_V26_WriteProbe(fd, oneSSeq0, sizeof(oneSSeq0), 5,  0x010, buffer);
    XBOX_V26_WriteProbe(fd, oneSSeq1, sizeof(oneSSeq1), 5,  0x020, buffer);
    XBOX_V26_WriteProbe(fd, oneSSeq1, sizeof(oneSSeq1), 64, 0x040, buffer);

    /* Endpoint-cancel/clear path, then retry the canonical packet. */
    s32 rc = USB_ClearHalt(fd, XBOX_V26_OUT_EP);
    if (rc >= 0)
        xboxV26Mask |= 0x080;
    usleep(15000);
    XBOX_V26_WriteProbe(fd, powerOn, sizeof(powerOn), 5, 0x100, buffer);

    /* Re-assert the known interface/alternate setting, then retry once. */
    rc = USB_SetAlternativeInterface(fd, 0, 0);
    if (rc >= 0)
        xboxV26Mask |= 0x200;
    usleep(15000);
    XBOX_V26_WriteProbe(fd, powerOn, sizeof(powerOn), 5, 0x400, buffer);
}

static inline s32 XBOX_V26_USB_OpenDevice(s32 device_id, u16 vid, u16 pid, s32 *fd)
{
    s32 rc = USB_OpenDevice(device_id, vid, pid, fd);
    if (rc < 0)
        return rc;

    if (vid == XBOX_V26_VID && pid == XBOX_V26_PID && fd && *fd != -1) {
        xboxV26Fd = *fd;
        xboxV26ProbeRan = false;
        xboxV26Mask = 0;

        /* Unlike V25, failures are diagnostic only; never close the device. */
        usb_devdesc desc;
        memset(&desc, 0, sizeof(desc));
        s32 descRc = USB_GetDescriptors(*fd, &desc);
        if (descRc >= 0) {
            xboxV26Mask |= 0x001;
            USB_FreeDescriptors(&desc);
        }

        u8 configuration = 0;
        s32 cfgRc = USB_GetConfiguration(*fd, &configuration);
        if (cfgRc >= 0)
            xboxV26Mask |= 0x002;
    }

    return rc;
}

static inline s32 XBOX_V26_USB_ReadIntrMsgAsync(s32 fd, u8 endpoint, u16 length,
                                                 void *data, usbcallback cb,
                                                 void *userdata)
{
    /* Queue the real read first, matching Linux xpad ordering. */
    s32 rc = USB_ReadIntrMsgAsync(fd, endpoint, length, data, cb, userdata);

    if (fd == xboxV26Fd && endpoint == XBOX_V26_IN_EP && rc >= 0) {
        XBOX_V26_RunProbe(fd);
        /* Preserve the real queued read but expose the whole probe mask in q:. */
        return 26000 + (s32)xboxV26Mask;
    }

    return rc;
}

/* Define wrappers only after their implementations so internal calls above
 * resolve to libogc2's real functions instead of recursing into this shim.
 */
#define USB_OpenDevice       XBOX_V26_USB_OpenDevice
#define USB_ReadIntrMsgAsync XBOX_V26_USB_ReadIntrMsgAsync

#endif
