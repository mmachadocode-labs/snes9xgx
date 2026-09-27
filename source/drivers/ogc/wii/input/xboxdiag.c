#ifdef HW_RVL

#include <gccore.h>
#include <ogc/usb.h>
#include <stdio.h>
#include <string.h>

#define MAX_USB_DEVICES 32
#define MICROSOFT_VID 0x045e

char* XBOXONE_Status(void);

static void probeMicrosoftDevice(const usb_device_entry *dev, char *text, size_t textSize)
{
    s32 fd = -1;
    s32 rc = USB_OpenDevice(dev->device_id, dev->vid, dev->pid, &fd);
    if (rc < 0)
    {
        snprintf(text, textSize, "MS %04x:%04x open:%d", dev->vid, dev->pid, rc);
        return;
    }

    usb_devdesc desc;
    rc = USB_GetDescriptors(fd, &desc);
    if (rc < 0)
    {
        snprintf(text, textSize, "MS %04x:%04x desc:%d", dev->vid, dev->pid, rc);
        USB_CloseDevice(&fd);
        return;
    }

    u8 epIn = 0;
    u8 epOut = 0;
    u8 cfg = 0;
    u8 intfClass = 0;
    u8 intfSubClass = 0;
    u8 intfProtocol = 0;

    if (desc.configurations)
    {
        for (u8 c = 0; c < desc.bNumConfigurations && !(epIn && epOut); ++c)
        {
            usb_configurationdesc *configuration = &desc.configurations[c];
            if (!configuration->interfaces)
                continue;

            for (u8 i = 0; i < configuration->bNumInterfaces && !(epIn && epOut); ++i)
            {
                usb_interfacedesc *intf = &configuration->interfaces[i];
                if (!intf->endpoints)
                    continue;

                u8 localIn = 0;
                u8 localOut = 0;
                for (u8 e = 0; e < intf->bNumEndpoints; ++e)
                {
                    usb_endpointdesc *ep = &intf->endpoints[e];
                    if ((ep->bmAttributes & 0x03) != USB_ENDPOINT_INTERRUPT)
                        continue;

                    if (ep->bEndpointAddress & 0x80)
                        localIn = ep->bEndpointAddress;
                    else
                        localOut = ep->bEndpointAddress;
                }

                if (localIn && localOut)
                {
                    epIn = localIn;
                    epOut = localOut;
                    cfg = configuration->bConfigurationValue;
                    intfClass = intf->bInterfaceClass;
                    intfSubClass = intf->bInterfaceSubClass;
                    intfProtocol = intf->bInterfaceProtocol;
                }
            }
        }
    }

    if (epIn && epOut)
    {
        snprintf(text, textSize, "MS %04x:%04x ep:%02x/%02x c%u %02x/%02x/%02x",
                 dev->vid, dev->pid, epIn, epOut, cfg,
                 intfClass, intfSubClass, intfProtocol);
    }
    else
    {
        snprintf(text, textSize, "MS %04x:%04x no int ep", dev->vid, dev->pid);
    }

    USB_FreeDescriptors(&desc);
    USB_CloseDevice(&fd);
}

char* XBOXONE_DiagnosticStatus(void)
{
    static char text[96];
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

    for (u8 i = 0; i < count; ++i)
    {
        if (devices[i].vid == MICROSOFT_VID)
        {
            probeMicrosoftDevice(&devices[i], text, sizeof(text));
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
