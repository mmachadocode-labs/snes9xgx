#ifndef SNES9XGX_XBOX_V25_USB_HOOK_H
#define SNES9XGX_XBOX_V25_USB_HOOK_H

/*
 * V28: direct IOS58 /dev/usb/ven lifecycle probe for Xbox One S 045e:02ea.
 *
 * V27 proved that libogc2 can list and "open" all three VEN interface entries,
 * but GetDeviceInfo/control operations and every interrupt OUT still fail.
 * WiiBrew documents an explicit VEN Attach (ioctl 4) step before Resume,
 * GetDeviceInfo and transfers. libogc2's V5 USB_OpenDevice path does not issue
 * Attach, and also ignores the result of USB_ResumeDevice().
 *
 * This probe opens a second VEN handle, obtains its own device list, explicitly
 * attaches interface 0, resumes it, gets device parameters, then submits the
 * two Xbox One S GIP init packets directly through VEN ioctlv 0x13. This avoids
 * libogc2's high-level V5 lifecycle while retaining the normal driver for the
 * rest of the emulator.
 *
 * q: in Credits is replaced with 28000 + success mask.
 */

#include <ogc/usb.h>
#include <ogc/system.h>
#include <ogc/timesupp.h>
#include <ipc.h>
#include <stdbool.h>
#include <string.h>

#define XBOX_V28_VID 0x045e
#define XBOX_V28_PID 0x02ea
#define XBOX_V28_IN_EP  0x82
#define XBOX_V28_OUT_EP 0x02

#define V28_IOCTL_GETVERSION       0
#define V28_IOCTL_GETDEVICECHANGE  1
#define V28_IOCTL_GETDEVPARAMS     3
#define V28_IOCTL_ATTACH           4
#define V28_IOCTL_RELEASE          5
#define V28_IOCTL_ATTACHFINISH     6
#define V28_IOCTL_SUSPEND_RESUME  16
#define V28_IOCTL_INTRMSG         19

/* q = 28000 + mask
 * 0x0001 IOS_Open(/dev/usb/ven, handle 1) succeeded
 * 0x0002 GetVersion returned 0x50001
 * 0x0004 GetDeviceChange succeeded
 * 0x0008 found 045e:02ea interface 0 on our handle
 * 0x0010 Attach(interface 0) succeeded
 * 0x0020 Resume succeeded
 * 0x0040 GetDeviceInfo succeeded
 * 0x0080 direct power-on interrupt OUT succeeded
 * 0x0100 direct One-S init interrupt OUT succeeded
 * 0x0200 AttachFinish succeeded
 * 0x0400 power-on OUT after AttachFinish succeeded
 * 0x0800 Release succeeded
 * 0x1000 normal libogc2 USB_GetDeviceList succeeded (sanity)
 * 0x2000 direct probe MEM2 allocation succeeded
 */
static s32 xboxV28DriverFd = -1;
static bool xboxV28ProbeRan = false;
static u32 xboxV28Mask = 0;

static const char xboxV28VenPath[] ATTRIBUTE_ALIGN(32) = "/dev/usb/ven";

/* Layout required in the first 64-byte vector for VEN IntrTransfer.
 * Only offsets 0, 8, 12 and 14 are relevant to the IOS USB_VEN wrapper.
 */
static inline s32 XBOX_V28_DirectIntrOut(s32 venFd, s32 deviceId, u8 endpoint,
                                        const u8 *packet, u16 packetLen,
                                        u8 *args64, u8 *data64)
{
    memset(args64, 0, 64);
    memset(data64, 0, 64);
    memcpy(data64, packet, packetLen);

    *(s32 *)(args64 + 0) = deviceId;
    *(void **)(args64 + 8) = data64;
    *(u16 *)(args64 + 12) = packetLen;
    *(u8  *)(args64 + 14) = endpoint;

    ioctlv vec[2];
    vec[0].data = args64;
    vec[0].len = 64;
    vec[1].data = data64;
    vec[1].len = packetLen;

    return IOS_Ioctlv(venFd, V28_IOCTL_INTRMSG, 2, 0, vec);
}

static inline void XBOX_V28_RunProbe(void)
{
    if (xboxV28ProbeRan)
        return;
    xboxV28ProbeRan = true;

    /* One allocation keeps every IOS-visible buffer in MEM2 and 32-byte aligned. */
    u8 *block = (u8 *)SYS_AllocArenaMem2Lo(0x400, 32);
    if (!block)
        return;
    xboxV28Mask |= 0x2000;

    u8 *ver      = block + 0x000; /* 0x20 */
    u8 *listBuf  = block + 0x020; /* 0x180 */
    u8 *cmd      = block + 0x1a0; /* 0x20 */
    u8 *devInfo  = block + 0x1c0; /* 0xc0 */
    u8 *args64   = block + 0x280; /* 0x40 */
    u8 *data64   = block + 0x2c0; /* 0x40 */

    /* Sanity-check that libogc2 still sees the VEN list. */
    static usb_device_entry sanity[32] ATTRIBUTE_ALIGN(32);
    u8 sanityCount = 0;
    memset(sanity, 0, sizeof(sanity));
    if (USB_GetDeviceList(sanity, 32, 0xff, &sanityCount) >= 0)
        xboxV28Mask |= 0x1000;

    /* VEN interprets IOS_Open's mode as a handle ID. libogc2 owns handle 0;
     * use handle 1 for an isolated lifecycle test.
     */
    s32 venFd = IOS_Open(xboxV28VenPath, 1);
    if (venFd < 0)
        return;
    xboxV28Mask |= 0x0001;

    memset(ver, 0, 0x20);
    if (IOS_Ioctl(venFd, V28_IOCTL_GETVERSION, NULL, 0, ver, 0x20) == 0 &&
        *(u32 *)ver == 0x00050001)
        xboxV28Mask |= 0x0002;

    memset(listBuf, 0, 0x180);
    s32 listRc = IOS_Ioctl(venFd, V28_IOCTL_GETDEVICECHANGE,
                           NULL, 0, listBuf, 0x180);
    if (listRc < 0) {
        IOS_Close(venFd);
        return;
    }
    xboxV28Mask |= 0x0004;

    usb_device_entry *entries = (usb_device_entry *)listBuf;
    s32 targetId = -1;
    int count = listRc;
    if (count > 32) count = 32;

    for (int i = 0; i < count; ++i) {
        if (entries[i].vid == XBOX_V28_VID && entries[i].pid == XBOX_V28_PID) {
            u8 interfaceNumber = (u8)((entries[i].token >> 8) & 0xff);
            if (interfaceNumber == 0) {
                targetId = entries[i].device_id;
                xboxV28Mask |= 0x0008;
                break;
            }
        }
    }

    if (targetId == -1) {
        IOS_Ioctl(venFd, V28_IOCTL_ATTACHFINISH, NULL, 0, NULL, 0);
        IOS_Close(venFd);
        return;
    }

    /* Attach the interface to this VEN handle. */
    memset(cmd, 0, 0x20);
    *(s32 *)(cmd + 0) = targetId;
    if (IOS_Ioctl(venFd, V28_IOCTL_ATTACH, cmd, 0x20, NULL, 0) >= 0)
        xboxV28Mask |= 0x0010;

    /* Resume: libogc2 writes the state in byte 11 via buf[2] on PPC. */
    memset(cmd, 0, 0x20);
    *(s32 *)(cmd + 0) = targetId;
    ((s32 *)cmd)[2] = 1;
    if (IOS_Ioctl(venFd, V28_IOCTL_SUSPEND_RESUME, cmd, 0x20, NULL, 0) >= 0)
        xboxV28Mask |= 0x0020;

    /* GetDeviceInfo is documented as mandatory before using the device. */
    memset(cmd, 0, 0x20);
    *(s32 *)(cmd + 0) = targetId;
    memset(devInfo, 0, 0xc0);
    if (IOS_Ioctl(venFd, V28_IOCTL_GETDEVPARAMS,
                  cmd, 0x20, devInfo, 0xc0) >= 0)
        xboxV28Mask |= 0x0040;

    static const u8 powerOn[]  = { 0x05, 0x20, 0x00, 0x01, 0x00 };
    static const u8 oneSInit[] = { 0x05, 0x20, 0x01, 0x0f, 0x06 };

    if (XBOX_V28_DirectIntrOut(venFd, targetId, XBOX_V28_OUT_EP,
                               powerOn, sizeof(powerOn), args64, data64) >= 0)
        xboxV28Mask |= 0x0080;
    udelay(15000);

    if (XBOX_V28_DirectIntrOut(venFd, targetId, XBOX_V28_OUT_EP,
                               oneSInit, sizeof(oneSInit), args64, data64) >= 0)
        xboxV28Mask |= 0x0100;
    udelay(15000);

    if (IOS_Ioctl(venFd, V28_IOCTL_ATTACHFINISH, NULL, 0, NULL, 0) >= 0)
        xboxV28Mask |= 0x0200;

    if (XBOX_V28_DirectIntrOut(venFd, targetId, XBOX_V28_OUT_EP,
                               powerOn, sizeof(powerOn), args64, data64) >= 0)
        xboxV28Mask |= 0x0400;

    memset(cmd, 0, 0x20);
    *(s32 *)(cmd + 0) = targetId;
    if (IOS_Ioctl(venFd, V28_IOCTL_RELEASE, cmd, 0x20, NULL, 0) >= 0)
        xboxV28Mask |= 0x0800;

    IOS_Close(venFd);
}

static inline s32 XBOX_V28_USB_OpenDevice(s32 device_id, u16 vid, u16 pid, s32 *fd)
{
    s32 rc = USB_OpenDevice(device_id, vid, pid, fd);
    if (rc < 0)
        return rc;

    if (vid == XBOX_V28_VID && pid == XBOX_V28_PID && fd && *fd != -1) {
        xboxV28DriverFd = *fd;
        xboxV28ProbeRan = false;
        xboxV28Mask = 0;
    }
    return rc;
}

static inline s32 XBOX_V28_USB_ReadIntrMsgAsync(s32 fd, u8 endpoint, u16 length,
                                                 void *data, usbcallback cb,
                                                 void *userdata)
{
    if (fd == xboxV28DriverFd && endpoint == XBOX_V28_IN_EP && !xboxV28ProbeRan)
        XBOX_V28_RunProbe();

    s32 rc = USB_ReadIntrMsgAsync(fd, endpoint, length, data, cb, userdata);

    if (fd == xboxV28DriverFd && endpoint == XBOX_V28_IN_EP && rc >= 0)
        return 28000 + (s32)xboxV28Mask;

    return rc;
}

#define USB_OpenDevice       XBOX_V28_USB_OpenDevice
#define USB_ReadIntrMsgAsync XBOX_V28_USB_ReadIntrMsgAsync

#endif
