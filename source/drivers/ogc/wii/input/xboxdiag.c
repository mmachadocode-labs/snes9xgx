#ifdef HW_RVL

/*
 * Keep the Credits diagnostics side-effect free.
 *
 * Earlier development versions reopened the controller and requested USB
 * descriptors from this function. That meant merely opening Credits could
 * race the real input driver and also used stack-backed IOS IPC structures.
 * The input driver now owns all USB access and exposes its current state here.
 */

char* XBOXONE_Status(void);

char* XBOXONE_DiagnosticStatus(void)
{
    return XBOXONE_Status();
}

#endif
