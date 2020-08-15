/*
This sample program demonstrates how to be informed about the removal of a device.

  Attention:
  If you build this sample in debug mode and run it using a GigE camera device, pylon will set the heartbeat
  timeout to 5 minutes. This is done to allow debugging and single stepping of the code without
  the camera thinking we're hung because we don't send any heartbeats.
  This also means that it would normally take 5 minutes for the application to notice that a GigE device
  has been disconnected.

  To work around this, the heartbeat timeout will be set to 1000 ms before we remove a device and wait to
  notice the removal.

*/

#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>

#include <pylonc/PylonC.h>

#ifdef __GNUC__
#   include <unistd.h>
#   define Sleep(ms) usleep(ms*1000)
#endif


static int callbackCounter = 0;  /* Will be incremented by the callback function. */

/* Simple error handling */
#define CHECK( errc ) if ( GENAPI_E_OK != errc ) printErrorAndExit( errc )

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void);

/* This method demonstrates how to retrieve the error message
for the last failed function call. */
void printErrorAndExit( GENAPIC_RESULT errc );


/* This function will be registered as a callback function that is called
when the opened device has been removed. On Windows only stdcall functions can be registered. */
void GENAPIC_CC removalCallbackFunction(PYLON_DEVICE_HANDLE hDevice );


/* Sets the heartbeat timeout. */
int64_t setHeartbeatTimeout( PYLON_DEVICE_HANDLE hDevice, int64_t timeout_ms );

int main(void)
{
    GENAPIC_RESULT              res;                /* Return value of pylon methods. */
    size_t                      numDevices;         /* Number of available devices.   */
    PYLON_DEVICE_HANDLE         hDev;               /* Handle for the pylon device.   */
    PYLON_DEVICECALLBACK_HANDLE hCb;                /* Required for deregistering the callback. */
    int                         loopCount;          /* Counter. */
    int                         isGigECamera;       /* 1 if the device is a GigE device. */

    /* Before using any pylon methods, the pylon runtime must be initialized. */
    PylonInitialize();

    /* Enumerate all camera devices. You must call
    PylonEnumerateDevices() before creating a device. */
    res = PylonEnumerateDevices( &numDevices );
    CHECK(res);
    if ( 0 == numDevices )
    {
        fprintf( stderr, "No devices found.\n" );
        /* Before exiting a program, PylonTerminate() should be called to release
        all pylon related resources. */
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_FAILURE);
    }

    /* Get a handle for the first device found.  */
    res = PylonCreateDeviceByIndex( 0, &hDev );
    CHECK(res);

    /* Before using the device, it must be opened. Open it for configuring
    parameters and for grabbing images. */
    res = PylonDeviceOpen( hDev, PYLONC_ACCESS_MODE_CONTROL | PYLONC_ACCESS_MODE_STREAM );
    CHECK(res);


    /* Print out the name of the camera we are using. */
    {
        char buf[256];
        size_t siz = sizeof(buf);
        _Bool isReadable;

        isReadable = PylonDeviceFeatureIsReadable(hDev, "DeviceModelName");
        if ( isReadable )
        {
            res = PylonDeviceFeatureToString( hDev, "DeviceModelName", buf, &siz );
            CHECK(res);
            printf("Using camera %s\n", buf);
        }
    }

    /* Register the callback function. */
    res = PylonDeviceRegisterRemovalCallback( hDev, removalCallbackFunction, &hCb );
    CHECK(res);


    /* For GigE cameras, the application periodically sends heartbeat signals to the camera to keep the
    connection to the camera alive. If the camera doesn't receive heartbeat signals within the time
    period specified by the heartbeat timeout counter, the camera resets the connection.
    When the application is stopped by the debugger, the application cannot create the heartbeat signals.
    For that reason, the pylon runtime extends the heartbeat timeout when debugging to 5 minutes to allow
    debugging. For GigE cameras, we will set the heartbeat timeout to a shorter period before testing the
    callbacks.
    The heartbeat mechanism is also used for detection of device removal. When the pylon runtime doesn't
    receive an acknowledge for the heartbeat signal, it is assumed that the device has been removed. A
    removal callback will be fired in that case.
    By decreasing the heartbeat timeout, the surprise removal will be noticed earlier. */
    {
        /* Find out if we are using a GigE camera. */
        PylonDeviceInfo_t devInfo;
        res = PylonDeviceGetDeviceInfo( hDev, &devInfo );
        CHECK(res);
        isGigECamera = 0 == strcmp( devInfo.DeviceClass, "BaslerGigE" );

        /* Adjust the heartbeat timeout. */
        if ( isGigECamera )
        {
            setHeartbeatTimeout( hDev, 1000 );  /* 1000 ms */
        }
    }

    /* Ask the user to disconnect a device. */
    loopCount = 20 * 4;
    printf( "Please disconnect the device (timeout %d s) \n", loopCount / 4 );


    /* Wait until the removal has been noticed and callback function has been fired. */
    do
    {
        /* Print a . every few seconds to tell the user we're waiting for the callback. */
        if (--loopCount % 4 == 0)
        {
            printf(".");
            fflush(stdout);
        }

        Sleep(250);
    }
    while (callbackCounter < 1 && loopCount >= 0);  /*  Check loopCount so we won't wait forever. */


    if (callbackCounter < 1)
        printf( "\nTimeout expired. Device hasn't been removed.\n" );


    /* Clean up. */

    /* ... Deregister the removal callback. */
    res = PylonDeviceDeregisterRemovalCallback( hDev, hCb );
    CHECK(res);

    /* ....Close and release the pylon device. */
    res = PylonDeviceClose( hDev );
    CHECK(res);
    res = PylonDestroyDevice ( hDev );
    CHECK(res);


    /* Shut down the pylon runtime system. Don't call any pylon method after
       calling PylonTerminate(). */
    PylonTerminate();
    pressEnterToExit();

    return EXIT_SUCCESS;
}



/* The function to be called when the removal of an opened device is detected. */
void GENAPIC_CC removalCallbackFunction(PYLON_DEVICE_HANDLE hDevice )
{
    PylonDeviceInfo_t   di;
    GENAPIC_RESULT      res;

    /* Print out the name of the device. It is not possible to read the name
    from the camera since it has been removed. Use the device's device
    information instead. For accessing the device information, no reading from
    the device is required. */

    /* Retrieve the device information for the removed device. */
    res = PylonDeviceGetDeviceInfo( hDevice, &di );
    CHECK(res);


    /* Print out the name. */
    printf( "\nCallback function for removal of device %s (%s).\n", di.FriendlyName, di.FullName );

    /* Increment the counter to indicate that the callback has been fired. */
    callbackCounter++;
}



/* If the device provides a heartbeat timeout, this function will set the heartbeat timeout.
   When the device provides the parameter, the old value is returned, -1 otherwise.
   The heartbeat timeout is a parameter provided by the transport layer.
   The transport layer parameters are exposed as a GenApi node map that
   can be retrieved from the device.
*/
int64_t setHeartbeatTimeout( PYLON_DEVICE_HANDLE hDev, int64_t timeout_ms )
{

    NODEMAP_HANDLE              hNodemap;   /* Handle to the node map */
    NODE_HANDLE                 hNode;      /* Handle to a node, i.e., a feature */
    GENAPIC_RESULT              res;        /* Return value */
    int64_t                     oldTimeout; /* The current timeout value */

    /* Get the node map for the transport layer parameters. */
    res = PylonDeviceGetTLNodeMap( hDev, &hNodemap );
    CHECK(res);
    if ( GENAPIC_INVALID_HANDLE == hNodemap )
    {
        /* The device doesn't provide a transport layer node map. Nothing to do. */
        return -1;
    }
    /* Get the node for the heartbeat timeout parameter. */
    res = GenApiNodeMapGetNode( hNodemap, "HeartbeatTimeout", &hNode );
    CHECK(res);
    if ( GENAPIC_INVALID_HANDLE == hNode )
    {
        /* There is no heartbeat timeout parameter. Nothing to do. */
        return -1;
    }

    /* Get the current value. */
    res = GenApiIntegerGetValue( hNode, &oldTimeout );
    CHECK(res);

    /* Set the new value. */
    res = GenApiIntegerSetValue( hNode, timeout_ms );
    CHECK(res);

    /* Return the old value. */
    return oldTimeout;
}

/* This function demonstrates how to retrieve the error message for the last failed
   function call. */
void printErrorAndExit( GENAPIC_RESULT errc )
{
    char *errMsg;
    size_t length;

    /* Retrieve the error message.
    ... Find out first how big the buffer must be, */
    GenApiGetLastErrorMessage( NULL, &length );
    errMsg = (char*) malloc( length );
    /* ... and retrieve the message. */
    GenApiGetLastErrorMessage( errMsg, &length );

    fprintf( stderr, "%s (%#08x).\n", errMsg, (unsigned int) errc);
    free( errMsg);

    /* Retrieve more details about the error.
    ... Find out first how big the buffer must be, */
    GenApiGetLastErrorDetail( NULL, &length );
    errMsg = (char*) malloc( length );
    /* ... and retrieve the message. */
    GenApiGetLastErrorDetail( errMsg, &length );

    fprintf( stderr, "%s\n", errMsg);
    free( errMsg);

    PylonTerminate();  /* Releases all pylon resources. */
    pressEnterToExit();

    exit(EXIT_FAILURE);
}

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void)
{
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
}

