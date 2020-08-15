#include "MyBconAdapterLogging.h"
#include <stdio.h>
#include <stdarg.h>


// global function pointer for log function
static BconTraceFunc g_pTraceFunc = NULL;



// Write log output using the log function set with SetExternalLogFunction
EXTERN_C void BCON_ADAPTER_CDECL LogOutput(BconAdapterTraceLevel level, const char *pFormat, ...)
{
    if (g_pTraceFunc == NULL)
    {
        return;
    }

    va_list argptr;
    va_start(argptr, pFormat);

    BconTraceFunc pTraceFunc = g_pTraceFunc;
    if (pTraceFunc != NULL)
    {
        pTraceFunc(level, pFormat, argptr);
    }

    va_end(argptr);
}


// Set log function pointer, handed down in BconAdapterInit()
EXTERN_C void SetExternalLogFunction(BconTraceFunc pFunc)
{
    g_pTraceFunc = pFunc;
}

