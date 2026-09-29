#include <gccore.h>
#include <ogc/usb.h>
#include <wiiuse/wpad.h>

#include <stdio.h>
#include <string.h>

#define MICROSOFT_VID  0x045e
#define XBOX_ONE_S_PID 0x02ea
#define MAX_USB_DEVICES 32

static usb_device_entry devices[MAX_USB_DEVICES] ATTRIBUTE_ALIGN(32);
static u8 deviceCount = 0;
static s32 rcInit = 999;
static s32 rcList = 999;
static bool xboxFound = false;
static int refreshes = 0;

static void refreshDeviceList(void)
{
    memset(devices, 0, sizeof(devices));
    deviceCount = 0;
    rcList = USB_GetDeviceList(devices, MAX_USB_DEVICES, 0, &deviceCount);

    xboxFound = false;
    if (rcList >= 0) {
        for (u8 i = 0; i < deviceCount; ++i) {
            if (devices[i].vid == MICROSOFT_VID && devices[i].pid == XBOX_ONE_S_PID) {
                xboxFound = true;
                break;
            }
        }
    }
    refreshes++;
}

static void drawScreen(void)
{
    printf("\x1b[2J\x1b[H");
    printf("USB ENUMERATOR - Wii / libogc2\n");
    printf("Target Xbox One S: 045e:02ea\n");
    printf("HOME: exit\n\n");

    printf("USB_Initialize: %d\n", rcInit);
    printf("USB_GetDeviceList: %d\n", rcList);
    printf("Devices/interfaces: %u   refresh: %d\n\n",
           (unsigned)deviceCount, refreshes);

    if (rcInit < 0) {
        printf("ERROR: USB_Initialize failed.\n");
        return;
    }

    if (rcList < 0) {
        printf("ERROR: USB_GetDeviceList failed.\n");
        return;
    }

    if (deviceCount == 0) {
        printf("No USB devices/interfaces visible yet.\n");
    } else {
        printf(" #   device_id     VID:PID   token\n");
        printf("----------------------------------------\n");

        int shown = deviceCount;
        if (shown > 20)
            shown = 20;

        for (int i = 0; i < shown; ++i) {
            usb_device_entry *d = &devices[i];
            bool target = (d->vid == MICROSOFT_VID && d->pid == XBOX_ONE_S_PID);
            printf("%c%-2d  %11d  %04x:%04x  %08x\n",
                   target ? '>' : ' ', i, d->device_id,
                   d->vid, d->pid, (unsigned)d->token);
        }

        if (deviceCount > 20)
            printf("... %u more entries not shown\n", (unsigned)(deviceCount - 20));
    }

    printf("\nXbox 045e:02ea: %s\n", xboxFound ? "FOUND" : "NOT FOUND");

    if (xboxFound)
        printf("Detection stage PASSED. No device was opened or modified.\n");
    else
        printf("Connect/disconnect the controller and watch this screen.\n");
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    VIDEO_Init();
    WPAD_Init();

    GXRModeObj *rmode = VIDEO_GetPreferredMode(NULL);
    void *xfb = MEM_K0_TO_K1(SYS_AllocateFramebuffer(rmode));
    console_init(xfb, 20, 20, rmode->fbWidth, rmode->xfbHeight,
                 rmode->fbWidth * VI_DISPLAY_PIX_SZ);

    VIDEO_Configure(rmode);
    VIDEO_SetNextFramebuffer(xfb);
    VIDEO_SetBlack(FALSE);
    VIDEO_Flush();
    VIDEO_WaitVSync();
    if (rmode->viTVMode & VI_NON_INTERLACE)
        VIDEO_WaitVSync();

    rcInit = USB_Initialize();

    int frame = 0;
    int redraw = 0;

    while (1) {
        WPAD_ScanPads();
        if (WPAD_ButtonsDown(0) & WPAD_BUTTON_HOME)
            break;

        if (rcInit >= 0 && (frame % 30) == 0)
            refreshDeviceList();

        if (++redraw >= 15) {
            redraw = 0;
            drawScreen();
        }

        frame++;
        VIDEO_WaitVSync();
    }

    USB_Deinitialize();
    return 0;
}
