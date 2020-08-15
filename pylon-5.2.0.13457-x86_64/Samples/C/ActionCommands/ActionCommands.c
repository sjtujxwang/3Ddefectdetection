/*
   This sample illustrates how to grab images
   using a GigE Vision action command to trigger multiple cameras.
   At least 2 connected GigE cameras are required for this sample.
*/

#ifndef _WIN32_WINNT
#   define _WIN32_WINNT 0x0400
#endif

#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <time.h>

#include <pylonc/PylonC.h>

/* Limits the amount of cameras used for grabbing.
It is important to manage the available bandwidth when grabbing with multiple
cameras. This applies, for instance, if two GigE cameras are connected to the
same network adapter via a switch. To manage the bandwidth, the GevSCPD
interpacket delay parameter and the GevSCFTD transmission delay parameter can
be set for each GigE camera device. The "Controlling Packet Transmission Timing
with the Interpacket and Frame Transmission Delays on Basler GigE Vision Cameras"
Application Note (AW000649xx000) provides more information about this topic. */
#define MAX_NUM_DEVICES 4

#define GIGE_PACKET_SIZE       1500 /* Size of one Ethernet packet. */
#define GIGE_PROTOCOL_OVERHEAD 36   /* Total number of bytes of protocol overhead. */

#define CHECK( errc ) if ( GENAPI_E_OK != errc ) printErrorAndExit( errc )

const uint32_t AllGroupMask = 0xffffffff;

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void);

/* This method demonstrates how to retrieve the error message for the last failed function call. */
void printErrorAndExit( GENAPIC_RESULT errc );

int main(void)
{
    GENAPIC_RESULT              res;                      /* Return value of pylon methods. */
    size_t                      i;                        /* Generic loop variable */
    size_t                      numDevicesEnumerated;     /* Number of available devices. */
    size_t                      numDevicesToUse;          /* Number of usable devices. */
    _Bool                       isAvail;                  /* Used for checking feature availability. */
    size_t                      enumeratedDeviceIndex;    /* Index of device when accessing the enumeration result. */
    size_t                      deviceIndex;              /* Index of device used in this sample. */
    PYLON_WAITOBJECTS_HANDLE    wos = NULL;               /* Wait objects. */
    uint32_t                    DeviceKey;                /* Random device key used in this session. It will be initialized below. */
    uint32_t                    GroupKey;                 /* Group key for the devices. In this sample all devices will be in the same group. */


    /* These are camera-specific variables. */
    PYLON_DEVICE_HANDLE         hDev[MAX_NUM_DEVICES];        /* Handle for the pylon device. */
    PYLON_STREAMGRABBER_HANDLE  hGrabber[MAX_NUM_DEVICES];    /* Handle for the pylon stream grabber. */
    unsigned char              *buffers[MAX_NUM_DEVICES];     /* Buffers used for grabbing. */
    PYLON_STREAMBUFFER_HANDLE   bufHandles[MAX_NUM_DEVICES];  /* Handles for the buffers. */
    PylonDeviceInfo_t           deviceInfos[MAX_NUM_DEVICES]; /* Information about enumerated devices */

    /* Seed the random number generator. */
    srand((unsigned int)time(NULL));

    /* Get random device key used in this session. */
    DeviceKey = rand();
    /* In this sample all cameras will belong to the same group. */
    GroupKey = 0x24;

    /* Before using any pylon methods, the pylon runtime must be initialized. */
    PylonInitialize();

    /* Enumerate all devices. You must call
    PylonEnumerateDevices() before creating a device. */
    res = PylonEnumerateDevices( &numDevicesEnumerated );
    CHECK(res);
    if ( numDevicesEnumerated == 0 )
    {
        fprintf( stdout, "No devices found!\n");
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_FAILURE);
    }

    /* Create wait objects so we can wait for the images of each camera */
    res = PylonWaitObjectsCreate(&wos);
    CHECK(res);

    /* =======================================================================
     *  Open Cameras and Set Parameters.
     * ======================================================================= */
    deviceIndex = 0;
    for (enumeratedDeviceIndex = 0; enumeratedDeviceIndex < numDevicesEnumerated; ++enumeratedDeviceIndex)
    {
        PylonDeviceInfo_t di;

        PylonGetDeviceInfo(deviceIndex, &di);

        res = PylonGetDeviceInfo( enumeratedDeviceIndex, &di);
        CHECK(res);

        if (strcmp(di.DeviceClass, "BaslerGigE") != 0)
        {
            /* Action commands are only supported by GigE cameras. */
            continue;
        }

        /* Get a handle for the device. */
        res = PylonCreateDeviceByIndex( enumeratedDeviceIndex, &hDev[deviceIndex] );
        CHECK(res);

        /* Before using the device, it must be opened. Open it for setting
        parameters and for grabbing images. */
        res = PylonDeviceOpen( hDev[deviceIndex], PYLONC_ACCESS_MODE_CONTROL | PYLONC_ACCESS_MODE_STREAM );
        CHECK(res);

        /* When the device has been opened successfully, we remember its deviceinfo
           so we can print out the name device name, etc. later */
        deviceInfos[deviceIndex] = di;

        /* Print out the name of the camera we are using. */
        printf("Using camera '%s'\n", deviceInfos[deviceIndex].ModelName);

        isAvail = PylonDeviceFeatureIsAvailable(hDev[deviceIndex], "ActionControl");
        if (! isAvail)
        {
            /* Action Command feature is not available. */
            fprintf(stderr, "Device doesn't support the Action Control");
            PylonTerminate();
            pressEnterToExit();
            exit( EXIT_SUCCESS );
        }

        /* Configure the first action (Action1)*/
        res = PylonDeviceSetIntegerFeatureInt32( hDev[deviceIndex], "ActionSelector", 1 );
        CHECK( res );
        res = PylonDeviceSetIntegerFeatureInt32( hDev[deviceIndex], "ActionDeviceKey", DeviceKey );
        CHECK( res );
        res = PylonDeviceSetIntegerFeatureInt32( hDev[deviceIndex], "ActionGroupKey", GroupKey );
        CHECK( res );
        res = PylonDeviceSetIntegerFeature( hDev[deviceIndex], "ActionGroupMask", AllGroupMask );
        CHECK( res );

        /* Set the trigger mode to FrameStart and TriggerSource to first action. */
        res = PylonDeviceFeatureFromString( hDev[deviceIndex], "TriggerSelector", "FrameStart" );
        CHECK(res);
        res = PylonDeviceFeatureFromString( hDev[deviceIndex], "TriggerMode", "On" );
        CHECK(res);
        res = PylonDeviceFeatureFromString( hDev[deviceIndex], "TriggerSource", "Action1" );  /* Action1 corresponds to ActionSelector=1. */
        CHECK(res);

        /* Set the pixel format to Mono8, where gray values will be output as 8-bit values for each pixel. */
        /* ... First check to see if the device supports the Mono8 format. */
        isAvail = PylonDeviceFeatureIsAvailable(hDev[deviceIndex], "EnumEntry_PixelFormat_Mono8");
        if ( ! isAvail )
        {
            /* Feature is not available. */
            fprintf(stderr, "Device doesn't support the Mono8 pixel format.\n");
            PylonTerminate();
            pressEnterToExit();
            exit (EXIT_SUCCESS);
        }

        /* ... Set the pixel format to Mono8. */
        res = PylonDeviceFeatureFromString( hDev[deviceIndex], "PixelFormat", "Mono8" );
        CHECK(res);

        /* For GigE cameras, we recommend increasing the packet size for better
        performance. When the network adapter supports jumbo frames, set the packet
        size to a value > 1500, e.g. to 8192. In this sample, we only set the packet size
        to 1500.

        Also we set the Inter-Packet and the Frame Transmission delay
        so that the switch can line up packets in a better way.
        */

        res = PylonDeviceSetIntegerFeature( hDev[deviceIndex], "GevSCPSPacketSize", GIGE_PACKET_SIZE );
        CHECK(res);

        res = PylonDeviceSetIntegerFeature( hDev[deviceIndex], "GevSCPD", (GIGE_PACKET_SIZE + GIGE_PROTOCOL_OVERHEAD)*(MAX_NUM_DEVICES-1) );
        CHECK(res);

        res = PylonDeviceSetIntegerFeature( hDev[deviceIndex], "GevSCFTD", (GIGE_PACKET_SIZE + GIGE_PROTOCOL_OVERHEAD) * deviceIndex );
        CHECK(res);

        /* One device created */
        ++deviceIndex;
    }

    /* Remember the number of devices actually created. */
    numDevicesToUse = deviceIndex;

    if (numDevicesToUse == 0)
    {
        fprintf(stderr, "No suitable cameras found!\n");
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_SUCCESS);
    }

    if (numDevicesToUse < 2)
    {
        printf("WARNING: This sample works best with two or more GigE cameras supporting action commands.\n");
    }

    /*  ======================================================================
        Allocate and Register Buffers for Grab.
        ====================================================================== */
    for (deviceIndex = 0; deviceIndex < numDevicesToUse; ++deviceIndex)
    {
        PYLON_WAITOBJECT_HANDLE hWait;
        int32_t payloadSize;


        /* Determine the required size of the grab buffer. */
        res = PylonDeviceGetIntegerFeatureInt32( hDev[deviceIndex], "PayloadSize", &payloadSize );
        CHECK(res);


        /* Image grabbing is done using a stream grabber.
          A device may be able to provide different streams. A separate stream grabber must
          be used for each stream. In this sample, we create a stream grabber for the default
          stream, i.e., the first stream ( index == 0 ).
          */

        /* Get the number of streams supported by the device and the transport layer. */
        res = PylonDeviceGetNumStreamGrabberChannels( hDev[deviceIndex], &i );
        CHECK(res);
        if ( i < 1 )
        {
            fprintf( stderr, "The transport layer doesn't support image streams.\n");
            PylonTerminate();
            pressEnterToExit();
            exit(EXIT_FAILURE);
        }

        /* Create and open a stream grabber for the first channel. */
        res = PylonDeviceGetStreamGrabber( hDev[deviceIndex], 0, &hGrabber[deviceIndex] );
        CHECK(res);
        res = PylonStreamGrabberOpen( hGrabber[deviceIndex] );
        CHECK(res);

        /* Get a handle for the stream grabber's wait object. The wait object
           allows waiting for buffers to be filled with grabbed data. */
        res = PylonStreamGrabberGetWaitObject( hGrabber[deviceIndex], &hWait );
        CHECK(res);

        /* Add the stream grabber's wait object to our wait objects.
           This is needed to be able to wait until at least one camera has
           grabbed an image in the grab loop below. */
        res = PylonWaitObjectsAdd(wos, hWait, NULL);
        CHECK(res);


        /* Allocate memory for grabbing.  */
        buffers[deviceIndex] = (unsigned char *) malloc ( payloadSize );
        if ( NULL == buffers[deviceIndex] )
        {
            fprintf( stderr, "Out of memory.\n" );
            PylonTerminate();
            pressEnterToExit();
            exit(EXIT_FAILURE);
        }

        /* We must tell the stream grabber the number and size of the buffers
            we are using. */
        /* .. We will not use more than NUM_BUFFERS for grabbing. */
        res = PylonStreamGrabberSetMaxNumBuffer( hGrabber[deviceIndex], 1 );
        CHECK(res);
        /* .. We will not use buffers bigger than payloadSize bytes. */
        res = PylonStreamGrabberSetMaxBufferSize( hGrabber[deviceIndex], payloadSize );
        CHECK(res);


        /*  Allocate the resources required for grabbing. After this, critical parameters
            that impact the payload size must not be changed until FinishGrab() is called. */
        res = PylonStreamGrabberPrepareGrab( hGrabber[deviceIndex] );
        CHECK(res);


        /* Before using the buffers for grabbing, they must be registered at
           the stream grabber. For each registered buffer, a buffer handle
           is returned. After registering, these handles are used instead of the
           raw pointers. */
        res = PylonStreamGrabberRegisterBuffer( hGrabber[deviceIndex], buffers[deviceIndex], payloadSize,  &bufHandles[deviceIndex] );
        CHECK(res);

        /* Feed the buffers into the stream grabber's input queue. For each buffer, the API
           allows passing in a pointer to additional context information. This pointer
           will be returned unchanged when the grab is finished. In our example, we use the index of the
           camera as context information. */
        res = PylonStreamGrabberQueueBuffer( hGrabber[deviceIndex], bufHandles[deviceIndex], (void *) deviceIndex );
        CHECK(res);
    }

    /* The stream grabber is now prepared. As soon as the camera starts acquiring images,
       the image data will be grabbed into the provided buffers.  */
    for (deviceIndex = 0; deviceIndex < numDevicesToUse; ++deviceIndex)
    {
        /* Let the camera acquire images. */
        res = PylonDeviceExecuteCommandFeature( hDev[deviceIndex], "AcquisitionStart");
        /* Do not call CHECK() here! Instead exit the loop. */
        if (res != GENAPI_E_OK)
        {
            break;
        }
    }


    /*  ======================================================================
        Issue an ActionCommand and Retrieve the Images.
        ====================================================================== */

    /* Only start the grab loop if all cameras have been "started" */
    if (res == GENAPI_E_OK)
    {
        char subnet[ 32 ] = {'\0'}; /* reserve space for a dotted ip address */
        size_t buflen = sizeof subnet;
        PYLON_DEVICE_INFO_HANDLE hDevInfo = NULL;

        /* Retrieve subnet broadcast address of first device.
           We must query this GigE specific value explicitly as it isn't contained
           in the generic PylonDeviceInfo_t struct. */
        res = PylonDeviceGetDeviceInfoHandle( hDev[0], &hDevInfo );
        CHECK( res );
        res = PylonDeviceInfoGetPropertyValueByName( hDevInfo, "SubnetAddress", subnet, &buflen );
        CHECK( res );


        /* Trigger the camera using an action command. */
        res = PylonGigEIssueActionCommand( DeviceKey, GroupKey, AllGroupMask, subnet, 0, NULL, NULL);
        CHECK( res );

        /* Grab one image from each camera. */
        for (deviceIndex = 0; deviceIndex < numDevicesToUse; ++deviceIndex)
        {
            _Bool isReady;
            size_t woIndex = 0;
            PylonGrabResult_t grabResult;

            /* Wait for the next buffer to be filled. Wait up to 5000 ms.*/
            res = PylonWaitObjectsWaitForAny(wos, 5000, &woIndex, &isReady );
            CHECK(res);
            if ( !isReady )
            {  /* Timeout occurred. */

                /* Grab timeout occurred. */
                fprintf(stderr, "Grab timeout occurred.\n");
                break; /* Stop grabbing. */
            }

            /* The woIndex corresponds to the index of the camera in handle arrays. */

            /* Retrieve the grab result. */
            res = PylonStreamGrabberRetrieveResult( hGrabber[woIndex], &grabResult, &isReady );
            CHECK(res);
            if ( !isReady )
            {
                /* Oops. No grab result available? We should never have reached this point.
                   Since the wait operation above returned without a timeout, a grab result
                   should be available. */
                fprintf(stderr, "Failed to retrieve a grab result\n");
                break;
            }

            /* Check to see whether the image was grabbed successfully. */
            if ( grabResult.Status == Grabbed && grabResult.PayloadType == PayloadType_Image )
            {
                /*  Success. Perform image processing. Since we passed more than one buffer
                to the stream grabber, the remaining buffers are filled while
                we are doing the image processing. The processed buffer won't be touched by
                the stream grabber until we pass it back to the stream grabber. */

                /* Pointer to the buffer attached to the grab result.
                   Get the buffer pointer from the result structure. Since we also got the buffer index,
                   we could alternatively use buffers[bufferIndex]. */
                /* Unsigned char* buffer = (unsigned char*) grabResult.pBuffer; */


                /* Perform processing. */
                printf("Grabbed frame from camera %u into buffer.\n", (unsigned int) woIndex);

#ifdef GENAPIC_WIN_BUILD
                /* Display image. */
                if (woIndex <= 31)
                {
                    res = PylonImageWindowDisplayImageGrabResult(woIndex, &grabResult);
                    CHECK(res);
                }
#endif
            }
            else if ( grabResult.Status == Failed )
            {
                /* If a buffer has been incompletely grabbed, the network bandwidth is possibly insufficient for transferring
                   multiple images simultaneously. See note on MAX_NUM_DEVICES. */
                fprintf( stderr, "Frame wasn't grabbed successfully.  Error code = 0x%08X\n",
                            grabResult.ErrorCode );
            }
        } /* for */
    }

    /* ========================================================================
        Clean up.
       ======================================================================== */

    /* Stop the image acquisition on the cameras. */

    for (deviceIndex = 0; deviceIndex < numDevicesToUse; ++deviceIndex)
    {
        /*  ... Stop the camera. */
        res = PylonDeviceExecuteCommandFeature(hDev[deviceIndex], "AcquisitionStop");
        CHECK(res);
    }

    /* Remove all wait objects from waitobjects. */
    res = PylonWaitObjectsRemoveAll(wos);
    CHECK(res);
    res = PylonWaitObjectsDestroy(wos);
    CHECK(res);


    /* Do the cleanup for each camera we've set up. */
    for (deviceIndex = 0; deviceIndex < numDevicesToUse; ++deviceIndex)
    {
        _Bool rdy;
        PylonGrabResult_t grabResult;

        /* Issue a cancel call to ensure that all queued buffers are put into the
           stream grabber's output queue. */
        res = PylonStreamGrabberCancelGrab( hGrabber[deviceIndex] );
        CHECK(res);

        /* The buffers can now be retrieved from the stream grabbers output queue. */
        do
        {
            res = PylonStreamGrabberRetrieveResult( hGrabber[deviceIndex], &grabResult, &rdy );
            CHECK(res);
        } while ( rdy );

        /* After all buffers have been retrieved from the stream grabber, they can be deregistered.
           After deregistering the buffers, it is safe to free the memory. */
        res = PylonStreamGrabberDeregisterBuffer( hGrabber[deviceIndex], bufHandles[deviceIndex] );
        CHECK(res);
        free( buffers[deviceIndex] );

        /* Release grabbing related resources. */
        res = PylonStreamGrabberFinishGrab( hGrabber[deviceIndex] );
        CHECK(res);

        /* When PylonStreamGrabberFinishGrab() has been called, parameters that impact the payload size (e.g.,
        the AOI width and height parameters) are unlocked and can be modified again. */

        /* Close the stream grabber. */
        res = PylonStreamGrabberClose( hGrabber[deviceIndex] );
        CHECK(res);


        /* Close and release the pylon device. The stream grabber becomes invalid
           after closing the pylon device. Don't call stream grabber related methods after
           closing or releasing the device. */
        res = PylonDeviceClose( hDev[deviceIndex] );
        CHECK(res);


        res = PylonDestroyDevice ( hDev[deviceIndex] );
        CHECK(res);
    }

    pressEnterToExit();

    /* ... Shut down the pylon runtime system. Don't call any pylon function after
       calling PylonTerminate(). */
    PylonTerminate();

    return EXIT_SUCCESS;

}

/* This method demonstrates how to retrieve the error message for the last failed
   function call. */
void printErrorAndExit( GENAPIC_RESULT errc )
{
    char *errMsg;
    char *errDetail;
    size_t length = 0;

    /* Retrieve the error message using double calling. */
    GenApiGetLastErrorMessage( NULL, &length );
    errMsg = (char*) malloc( length );
    GenApiGetLastErrorMessage( errMsg, &length );

    /* Get the detailed error message. */
    length = 0;
    GenApiGetLastErrorDetail( NULL, &length );
    errDetail = (char*) malloc( length );
    GenApiGetLastErrorDetail( errDetail, &length );


    fprintf( stderr, "%s (%#08x).\n%s\n", errMsg, (unsigned int) errc, errDetail);

    free(errDetail);
    free(errMsg);

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

