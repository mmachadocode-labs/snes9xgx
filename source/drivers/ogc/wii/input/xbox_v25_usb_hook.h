#ifndef SNES9XGX_XBOX_V25_USB_HOOK_H
#define SNES9XGX_XBOX_V25_USB_HOOK_H

/*
 * IOS58 /dev/usb/ven initialization shim for Xbox One S (045e:02ea).
 *
 * libogc2's USB_OpenDevice() resumes VEN devices but, unlike its HID path,
 * does not fetch the V5 device parameters/descriptors before normal traffic.
 * USB_GetDescriptors() performs USBV5_IOCTL_GETDEVPARAMS for VEN devices.
 *
 * Keep this wrapper Wii-only and Xbox-only. Other USB devices pass through
 * unchanged. The macro is defined after the wrapper so the call inside the
 * wrapper resolves to libogc2's real USB_OpenDevice().
 */

#include <ogc/usb.h>
#include <string.h>

static inline s32 XBOX_V25_USB_OpenDevice(s32 device_id, u16 vid, u16 pid, s32 *fd)
{
    s32 rc = USB_OpenDevice(device_id, vid, pid, fd);
    if (rc < 0)
        return rc;

    if (vid == 0x045e && pid == 0x02ea && fd && *fd != -1) {
        usb_devdesc desc;
        memset(&desc, 0, sizeof(desc));

        s32 descRc = USB_GetDescriptors(*fd, &desc);
        if (descRc < 0) {
            USB_CloseDevice(fd);
            return descRc;
        }

        USB_FreeDescriptors(&desc);
    }

    return rc;
}

#define USB_OpenDevice XBOX_V25_USB_OpenDevice

#endif
