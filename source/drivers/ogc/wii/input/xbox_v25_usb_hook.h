#ifndef SNES9XGX_XBOX_V25_USB_HOOK_H
#define SNES9XGX_XBOX_V25_USB_HOOK_H

/*
 * Legacy development shim. Global USB interception has been removed; the
 * direct IOS58 VEN driver lives entirely in xboxone.c. This header remains
 * force-included by the Wii Makefile for now and only provides the public USB
 * descriptor types/macros used by that C translation unit.
 */
#include <ogc/usb.h>

#endif
