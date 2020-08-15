#include <bconadapter/BconAdapterDefines.h>
#include <bconadapter/BconAdapterI2C.h>
#include "MyBconAdapterLogging.h"
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>


// Limit for string length
#define MAX_STRING_LENGTH  256

// Helper function to convert BconAdapterI2cBusHandle to file descriptor (int)
static int getFd(BconAdapterI2cBusHandle arg)
{
    return *((int*)(&arg));
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Opens the I2C bus connection to a camera device.
EXTERN_C BCON_ADAPTER_API BCONSTATUS BCON_ADAPTER_CALL BconAdapterI2cOpenConnection(
    const char deviceId[], BconAdapterI2cBusHandle *phBus, uint32_t *pDeviceAddress)
{
    // Copy string for usage
    char devicePath[MAX_STRING_LENGTH] = { 0 };
    strcpy(devicePath, deviceId);

    // String is like "/dev/i2c-2:99", so find ':'
    char *separator = (char*)strchr(devicePath, ':');
    if (separator == NULL)
    {
        LogOutput(TRACE_LEVEL_ERROR, "Device ID incomplete.");
        return BCON_E_NOT_FOUND;
    }

    // Open device
    separator[0] = '\0';
    int fp = open(devicePath, O_RDWR);
    if (fp <= 0)
    {
        LogOutput(TRACE_LEVEL_ERROR, "Could not open device.");
        return BCON_E_OPERATION_FAILED;
    }

    // Parse device address
    int deviceAddress = atoi(separator + 1);
    if ((deviceAddress < 0) || (deviceAddress >= 128))
    {
        LogOutput(TRACE_LEVEL_ERROR, "Error parsing device address, only 7-bit address allowed.");
        close(deviceAddress);
        return BCON_E_READ_FAILED;
    }

    // Return value device address
    if (pDeviceAddress != NULL)
    {
        *pDeviceAddress = deviceAddress;
    }

    // Return value bus handle
    if (phBus != NULL)
    {
        *phBus = (BconAdapterI2cBusHandle)(long)fp;
    }

    return BCON_OK;
}


//////////////////////////////////////////////////////////////////////////////
/// \brief Closes the I2C bus connection to a camera device.
EXTERN_C BCON_ADAPTER_API BCONSTATUS BCON_ADAPTER_CALL BconAdapterI2cCloseConnection(
    BconAdapterI2cBusHandle hBus, uint32_t deviceAddress)
{
    (void)deviceAddress;

    // Close file descriptor
    int ret = close(getFd(hBus));
    if (ret != 0)
    {
        LogOutput(TRACE_LEVEL_ERROR, "Could not close device.");
        return BCON_E_OPERATION_FAILED;
    }

    return BCON_OK;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Reads a block of data from I2C bus.
EXTERN_C BCON_ADAPTER_API BCONSTATUS BCON_ADAPTER_CALL BconAdapterI2cRead(
    BconAdapterI2cBusHandle hBus,
    uint32_t deviceAddress,
    void *pData,
    size_t sizeInBytes,
    size_t *pBytesRead,
    uint32_t timeout_ms)
{
    (void)timeout_ms;

    // Set target device address
    if (ioctl(getFd(hBus), I2C_SLAVE, deviceAddress) < 0)
    {
        LogOutput(TRACE_LEVEL_ERROR, "Error setting target address.");
        return BCON_E_OPERATION_FAILED;
    }

    // Read from I2C
    int bytesRead = read(getFd(hBus), pData, sizeInBytes);
    if (bytesRead <= 0)
    {
        return BCON_E_READ_FAILED;
    }
    else
    {
        *pBytesRead = bytesRead;
        return BCON_OK;
    }
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Writes a block of data on the I2C bus.
EXTERN_C BCON_ADAPTER_API BCONSTATUS BCON_ADAPTER_CALL BconAdapterI2cWrite(
    BconAdapterI2cBusHandle hBus,
    uint32_t deviceAddress,
    const void *pData,
    size_t sizeInBytes,
    uint32_t timeout_ms)
{
    (void)timeout_ms;

    // Set target device address
    if (ioctl(getFd(hBus), I2C_SLAVE, deviceAddress) < 0)
    {
        LogOutput(TRACE_LEVEL_ERROR, "Error setting target address.");
        return BCON_E_OPERATION_FAILED;
    }

    // Write to I2C
	// Note: The BCON camera uses clock stretching.
	// The used I2C master hardware must support clock stretching properly.
    int bytesWritten = write(getFd(hBus), pData, sizeInBytes);
    if (bytesWritten != sizeInBytes)
    {
        return BCON_E_WRITE_FAILED;
    }
    else
    {
        return BCON_OK;
    }
}

