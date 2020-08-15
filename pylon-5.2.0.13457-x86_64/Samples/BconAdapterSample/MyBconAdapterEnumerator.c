#include <bconadapter/BconAdapterEnumerator.h>
#include <bconadapter/BconAdapterDefines.h>
#include "MyBconAdapterLogging.h"
#include <assert.h>
#include <string.h>
#include <stdlib.h>


///////////////////////////////////////////////////////////////////////////
/// \brief Start device discovery.
EXTERN_C BCON_ADAPTER_API BCONSTATUS BconAdapterStartDiscovery(PFUNC_BCON_ADAPTER_DISCOVERY_CALLBACK callbackToBconAdapterUser, uintptr_t userCtx)
{
    BCONSTATUS returnCode = BCON_OK;

    // Get the I2C device configuration from environment variable BCON_ADAPTER_I2C_DEVICES
    //
    // Example for how to set BCON_ADAPTER_I2C_DEVICES for two devices: 
    //
    //     export BCON_ADAPTER_I2C_DEVICES="/dev/i2c-1:77 /dev/i2c-2:99"
    //
    // The two device identifiers /dev/i2c-1:77 and /dev/i2c-2:99 are separated by a blank.
    // The first device identifier /dev/i2c-1:77 consists of the I2C bus to open /dev/i2c-1  
    // and the device address 77, the bus and address parts being separated by a colon.
    // Analog for the second device identifier /dev/i2c-2:99.
    
    char *env_config = getenv("BCON_ADAPTER_I2C_DEVICES");

    if (env_config == NULL)
    {
        LogOutput(TRACE_LEVEL_ERROR, "Error reading env.var. ");
        return BCON_E_OPERATION_FAILED;
    }

    // Duplicate string for usage
    char *config = strdup(env_config);
    if (config == NULL)
    {
        LogOutput(TRACE_LEVEL_ERROR, "Error out of memory. ");
        return BCON_E_OPERATION_FAILED;
    }

    // Splitting the config string at the blanks yields the device identifier tokens.
    // Call the callback function for each token.
    char *token = strtok(config, " ");
    do
    {
        LogOutput(TRACE_LEVEL_INFORMATION, "Current Token is  _%s_", token);

        if (callbackToBconAdapterUser != NULL)
        {
            BCONSTATUS callbackToBconAdapterUserStatus = callbackToBconAdapterUser(token, (userCtx));
            if (!BCON_SUCCESS(callbackToBconAdapterUserStatus))
            {
                LogOutput(TRACE_LEVEL_ERROR, "Error calling BCON Adapter user callback in enumeration, status = 0x%08X", callbackToBconAdapterUserStatus);
                returnCode = callbackToBconAdapterUserStatus;
            }
        }

        token = strtok(NULL, " ");
    } 
    while (token != NULL);

    // free duplicate string created using strdup()
    free(config);

    return returnCode;
}

