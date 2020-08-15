#pragma once


#include <bconadapter/BconAdapterDefines.h>
#include <bconadapter/BconAdapterTypes.h>




// Trace level constants
#define TRACE_LEVEL_FATAL       (BconAdapterTraceLevel_Critical)      ///< Abnormal exit or termination
#define TRACE_LEVEL_ERROR       (BconAdapterTraceLevel_Error)         ///< Severe errors that need logging
#define TRACE_LEVEL_WARNING     (BconAdapterTraceLevel_Warning)       ///< Warnings such as allocation failure
#define TRACE_LEVEL_INFORMATION (BconAdapterTraceLevel_Information)   ///< Includes non-error cases(e.g. function entry or exit logging)
#define TRACE_LEVEL_VERBOSE     (BconAdapterTraceLevel_Verbose)       ///< Detailed traces from intermediate steps
#define TRACE_LEVEL_DEBUG       (BconAdapterTraceLevel_Debug)         ///< Traces for debugging purposes


// Write to log output 
EXTERN_C void BCON_ADAPTER_CDECL LogOutput(BconAdapterTraceLevel level, const char *pFormat, ...);

// Set log function pointer
EXTERN_C void SetExternalLogFunction(BconTraceFunc pFunc);



