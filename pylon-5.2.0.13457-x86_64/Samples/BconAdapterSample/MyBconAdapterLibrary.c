#include <bconadapter/BconAdapterApi.h>
#include <bconadapter/BconAdapterDefines.h>
#include "MyBconAdapterLogging.h"
#include <assert.h>
#include <string.h>
#include <stdlib.h>




///////////////////////////////////////////////////////////////////////////
/// \brief Init layer, returns BCON_OK on success.
EXTERN_C BCON_ADAPTER_API BCONSTATUS BconAdapterInit(BconTraceFunc pTraceFunc)
{
    SetExternalLogFunction(pTraceFunc);
    return BCON_OK;
}

///////////////////////////////////////////////////////////////////////////
/// \brief Deinit layer, return BCON_OK on success.
EXTERN_C BCON_ADAPTER_API BCONSTATUS BconAdapterExit(void)
{
    return BCON_OK;
}


///////////////////////////////////////////////////////////////////////////
/// \brief Returns version information.
EXTERN_C BCON_ADAPTER_API BCONSTATUS  BconAdapterGetVersion(
    unsigned int *apiMajorVersion,
    unsigned int *apiMinorVersion,
    unsigned int *adapterMajorVersion,
    unsigned int *adapterMinorVersion)
{
    assert(apiMajorVersion);
    assert(apiMinorVersion);
    assert(adapterMajorVersion);
    assert(adapterMinorVersion);

    *apiMajorVersion = BCON_ADAPTER_API_MAJORVERSION;
    *apiMinorVersion = BCON_ADAPTER_API_MINORVERSION;
    *adapterMajorVersion = 1;
    *adapterMinorVersion = 0;

    return BCON_OK;
}

