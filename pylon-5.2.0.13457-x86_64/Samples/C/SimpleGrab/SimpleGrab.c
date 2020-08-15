/*

This sample illustrates how to use the PylonDeviceGrabSingleFrame() convenience
method for grabbing images in a loop. PylonDeviceGrabSingleFrame() grabs one
single frame in single frame mode.

Grabbing in single frame acquisition mode is the easiest way to grab images. Note: in single frame
mode the maximum frame rate of the camera can't be achieved. The full frame
rate can be achieved by setting the camera to the continuous frame acquisition
mode and by grabbing in overlapped mode, i.e., image acquisition is done in parallel
with image processing. This is illustrated in the OverlappedGrab sample program.

*/

#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>

#include <pylonc/PylonC.h>

/* Simple error handling. */
#define CHECK( errc ) if ( GENAPI_E_OK != errc ) printErrorAndExit( errc )

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void);

/* This method demonstrates how to retrieve the error message
   for the last failed function call. */
void printErrorAndExit( GENAPIC_RESULT errc );

/* Calculating the minimum and maximum gray value of an image buffer */
void getMinMax( const unsigned char* pImg, int32_t width, int32_t height,
               unsigned char* pMin, unsigned char* pMax);


int main(void)
{
    GENAPIC_RESULT          res;           /* Return value of pylon methods. */
    size_t                  numDevices;    /* Number of available devices. */
    PYLON_DEVICE_HANDLE     hDev;          /* Handle for the pylon device. */
    const int               numGrabs = 10; /* Number of images to grab. */
    int32_t                 payloadSize;   /* Size of an image frame in bytes. */
    unsigned char*          imgBuf;        /* Buffer used for grabbing. */
    _Bool                    isAvail;
    int                     i;

    /* Before using any pylon methods, the pylon runtime must be initialized. */
    PylonInitialize();

    /* Enumerate all camera devices. You must call
    PylonEnumerateDevices() before creating a device! */
    res = PylonEnumerateDevices( &numDevices );
    CHECK(res);
    if ( 0 == numDevices )
    {
        fprintf( stderr, "No devices found!\n" );
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
            res = PylonDeviceFeatureToString(hDev, "DeviceModelName", buf, &siz );
            CHECK(res);
            printf("Using camera %s\n", buf);
        }
    }

    /* Set the pixel format to Mono8 if available, where gray values will be output as 8 bit values for each pixel. */
    isAvail = PylonDeviceFeatureIsAvailable(hDev, "EnumEntry_PixelFormat_Mono8");
    if (isAvail)
    {
        res = PylonDeviceFeatureFromString(hDev, "PixelFormat", "Mono8" );
        CHECK(res);
    }

    /* Disable acquisition start trigger if available */
    isAvail = PylonDeviceFeatureIsAvailable( hDev, "EnumEntry_TriggerSelector_AcquisitionStart");
    if (isAvail)
    {
        res = PylonDeviceFeatureFromString( hDev, "TriggerSelector", "AcquisitionStart");
        CHECK(res);
        res = PylonDeviceFeatureFromString( hDev, "TriggerMode", "Off");
        CHECK(res);
    }

    /* Disable frame burst start trigger if available. */
    isAvail = PylonDeviceFeatureIsAvailable( hDev, "EnumEntry_TriggerSelector_FrameBurstStart");
    if (isAvail)
    {
        res = PylonDeviceFeatureFromString( hDev, "TriggerSelector", "FrameBurstStart");
        CHECK(res);
        res = PylonDeviceFeatureFromString( hDev, "TriggerMode", "Off");
        CHECK(res);
    }

    /* Disable frame start trigger if available */
    isAvail = PylonDeviceFeatureIsAvailable( hDev, "EnumEntry_TriggerSelector_FrameStart");
    if (isAvail)
    {
        res = PylonDeviceFeatureFromString( hDev, "TriggerSelector", "FrameStart");
        CHECK(res);
        res = PylonDeviceFeatureFromString( hDev, "TriggerMode", "Off");
        CHECK(res);
    }

    /* For GigE cameras, we recommend increasing the packet size for better
       performance. If the network adapter supports jumbo frames, set the packet
       size to a value > 1500, e.g., to 8192. In this sample, we only set the packet size
       to 1500. */
    /* ... Check first to see if the GigE camera packet size parameter is supported
        and if it is writable. */
    isAvail = PylonDeviceFeatureIsWritable(hDev, "GevSCPSPacketSize");
    if ( isAvail )
    {
        /* ... The device supports the packet size feature. Set a value. */
        res = PylonDeviceSetIntegerFeature( hDev, "GevSCPSPacketSize", 1500 );
        CHECK(res);
    }

    /* Determine the required size of the grab buffer. */
    if ( PylonDeviceFeatureIsReadable(hDev, "PayloadSize") )
    {
        res = PylonDeviceGetIntegerFeatureInt32( hDev, "PayloadSize", &payloadSize );
        CHECK(res);
    }
    else
    {
        /* Note: Some camera devices, e.g Camera Link or BCON, don't have a payload size node.
                 In this case we'll look in the stream grabber node map for the PayloadSize node
                 The stream grabber, this can be a frame grabber, needs to be open to compute the
                 required payload size.
        */
        PYLON_STREAMGRABBER_HANDLE  hGrabber;
        NODEMAP_HANDLE hStreamNodeMap;
        NODE_HANDLE hNode;
        int64_t i64payloadSize;

        /* Temporary create and open a stream grabber for the first channel. */
        res = PylonDeviceGetStreamGrabber( hDev, 0, &hGrabber );
        CHECK(res);
        res = PylonStreamGrabberOpen( hGrabber );
        CHECK(res);

        res = PylonStreamGrabberGetNodeMap(hGrabber, &hStreamNodeMap);
        CHECK(res);
        res = GenApiNodeMapGetNode( hStreamNodeMap, "PayloadSize", &hNode );
        CHECK(res);
        if ( GENAPIC_INVALID_HANDLE == hNode )
        {
            fprintf( stderr, "There is no PayloadSize parameter.\n");
            PylonTerminate();
            pressEnterToExit();
            return EXIT_FAILURE;
        }
        res = GenApiIntegerGetValue(hNode, &i64payloadSize);
        CHECK(res);
        payloadSize = (int32_t) i64payloadSize;

        res = PylonStreamGrabberClose( hGrabber );
        CHECK(res);
    }

    /* Allocate memory for grabbing. */
    imgBuf = (unsigned char*) malloc( payloadSize );
    if ( NULL == imgBuf )
    {
        fprintf( stderr, "Out of memory.\n" );
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_FAILURE);
    }

    /* Grab some images in a loop. */
    for ( i = 0; i < numGrabs; ++i )
    {
        unsigned char min, max;
        PylonGrabResult_t grabResult;
        _Bool bufferReady;

        /* Grab one single frame from stream channel 0. The
        camera is set to single frame acquisition mode.
        Wait up to 500 ms for the image to be grabbed. */
        res = PylonDeviceGrabSingleFrame( hDev, 0, imgBuf, payloadSize,
            &grabResult, &bufferReady, 500 );
        if ( GENAPI_E_OK == res && !bufferReady )
        {
            /* Timeout occurred. */
            printf("Frame %d: timeout\n", i+1);
        }
        CHECK(res);

        /* Check to see if the image was grabbed successfully. */
        if ( grabResult.Status == Grabbed )
        {
            /* Success. Perform image processing. */
            getMinMax( imgBuf, grabResult.SizeX, grabResult.SizeY, &min, &max );
            printf("Grabbed frame #%2d. Min. gray value = %3u, Max. gray value = %3u\n", i+1, min, max);

#ifdef GENAPIC_WIN_BUILD
            /* Display image */
            res = PylonImageWindowDisplayImageGrabResult(0, &grabResult);
            CHECK(res);
#endif
        }
        else if ( grabResult.Status == Failed )
        {
            fprintf( stderr,  "Frame %d wasn't grabbed successfully.  Error code = 0x%08X\n",
                i+1, grabResult.ErrorCode );
        }
    }

    /* Clean up. Close and release the pylon device. */

    res = PylonDeviceClose( hDev );
    CHECK(res);
    res = PylonDestroyDevice ( hDev );
    CHECK(res);

    /* Free memory for grabbing. */
    free( imgBuf );

    pressEnterToExit();

    /* Shut down the pylon runtime system. Don't call any pylon method after
       calling PylonTerminate(). */
    PylonTerminate();

    return EXIT_SUCCESS;
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



/* Simple "image processing" function returning the minimum and maximum gray
   value of an 8 bit gray value image. */
void getMinMax( const unsigned char* pImg, int32_t width, int32_t height,
               unsigned char* pMin, unsigned char* pMax)
{
    unsigned char min = 255;
    unsigned char max = 0;
    unsigned char val;
    const unsigned char *p;

    for ( p = pImg; p < pImg + width * height; p++ )
    {
        val = *p;
        if ( val > max )
           max = val;
        if ( val < min )
           min = val;
    }
    *pMin = min;
    *pMax = max;
}

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void)
{
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
}

