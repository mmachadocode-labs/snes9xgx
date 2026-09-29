#ifndef SNES9XGX_XBOX_V25_USB_HOOK_H
#define SNES9XGX_XBOX_V25_USB_HOOK_H

/*
 * Legacy development shim intentionally left empty.
 *
 * The Xbox One S driver now owns its IOS58 /dev/usb/ven handle directly in
 * xboxone.c, so global interception of libogc USB calls is no longer needed.
 * The Makefile still force-includes this header for the moment; keeping it as a
 * no-op avoids changing unrelated USB users while the direct VEN driver is
 * validated on hardware.
 */

#endif
