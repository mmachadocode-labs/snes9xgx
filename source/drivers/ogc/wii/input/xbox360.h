#ifndef _XBOX360_H_
#define _XBOX360_H_

#include <gctypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The Wii frontend historically exposes USB Xbox controllers through the
 * XBOX360_* API. During development of Xbox One GIP support we keep that
 * public API intact and route it to the new driver.
 */
void XBOXONE_ScanPads();
u32 XBOXONE_ButtonsHeld(int chan);
char* XBOXONE_Status();
char* XBOXONE_DiagnosticStatus();

#define XBOX360_ScanPads    XBOXONE_ScanPads
#define XBOX360_ButtonsHeld XBOXONE_ButtonsHeld
#define XBOX360_Status      XBOXONE_DiagnosticStatus

#ifdef __cplusplus
}
#endif

#endif
