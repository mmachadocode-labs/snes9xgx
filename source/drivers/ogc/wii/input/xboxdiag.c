#ifdef HW_RVL

#include <gccore.h>
#include <ogc/usb.h>
#include <stdio.h>
#include <string.h>

#define MAX_USB_DEVICES 32
#define MICROSOFT_VID 0x045e

char* XBOXONE_Status(void);

char* XBOXONE_DiagnosticStatus(void)
{
    static char text[64];
    const char *driver = XBOXONE_Status();

    if (driver && strncmp(driver, "not found", 9) != 0)
    {
        snprintf(text, sizeof(text), "%s", driver);
        return text;
    }

    usb_device_entry devices[MAX_USB_DEVICES];
    memset(devices, 0, sizeof(devices));

    u8 count = 0;
    s32 rc = USB_GetDeviceList(devices, MAX_USB_DEVICES, 0, &count);
    if (rc < 0)
    {
        snprintf(text, sizeof(text), "list err:%d", rc);
        return text;
    }

    /* Prefer showing any Microsoft device, even if its PID is unexpected. */
    for (u8 i = 0; i < count; ++i)
    {
        if (devices[i].vid == MICROSOFT_VID)
        {
            snprintf(text, sizeof(text), "MS %04x:%04x (%u USB)",
                     devices[i].vid, devices[i].pid, count);
            return text;
        }
    }

    if (count == 0)
    {
        snprintf(text, sizeof(text), "0 USB");
        return text;
    }

    if (count == 1)
    {
        snprintf(text, sizeof(text), "USB1 %04x:%04x",
                 devices[0].vid, devices[0].pid);
        return text;
    }

    snprintf(text, sizeof(text), "USB%u %04x:%04x %04x:%04x",
             count,
             devices[0].vid, devices[0].pid,
             devices[1].vid, devices[1].pid);
    return text;
}

#endif
