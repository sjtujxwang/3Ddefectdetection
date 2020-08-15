/*
   This sample illustrates how to grab images and process images
   using multiple cameras simultaneously.

   The sample uses a pool of buffers that are passed to a stream grabber to be filled with
   image data. Once a buffer is filled and ready for processing, the buffer is retrieved from
   the stream grabber, processed, and passed back to the stream grabber to be filled again.
   Buffers retrieved from the stream grabber are not overwritten as long as
   they are not passed back to the stream grabber.
*/

#ifndef _WIN32_WINNT
#   define _WIN32_WINNT 0x0400
#endif

#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>

#include <pylonc/PylonC.h>

#ifdef GENAPIC_LINUX_BUILD
#   include <sys/timerfd.h>
#   include <alloca.h>
#   include <errno.h>
#   include <unistd.h>
#endif

/* Limits the amount of cameras used for grabbing.
   It is important to manage the available bandwidth when grabbing with multiple
   cameras. This applies, for instance, if two GigE cameras are connected to the
   same network adapter via a switch. To manage the bandwidth, the GevSCPD
   interpacket delay parameter and the GevSCFTD transmission delay parameter can
   be set for each GigE camera device. The "Controlling Packet Transmission Timing
   with the Interpacket and Frame Transmission Delays on Basler GigE Vision Cameras"
   Application Note (AW000649xx000) provides more information about this topic. */
#define NUM_DEVICES 2
#define NUM_BUFFERS 5         /* Number of buffers used for grabbing. */

#define GIGE_PACKET_SIZE       1500 /* Size of one Ethernet packet. */
#define GIGE_PROTOCOL_OVERHEAD 36   /* Total number of bytes of protocol overhead. */

#define CHECK( errc ) if ( GENAPI_E_OK != errc ) printErrorAndExit( errc )

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void);

/* This method demonstrates how to retrieve the error message for the last failed function call. */
void printErrorAndExit( GENAPIC_RESULT errc );

/* Calculating the minimum and maximum gray value. */
void getMinMax( const unsigned char* pImg, int32_t width, int32_t height,
               unsigned char* pMin, unsigned char* pMax);


int main(void)
{
    GENAPIC_RESULT              res;                      /* Return value of pylon methods. */
    size_t                      numDevicesAvail;          /* Number of available devices. */
    _Bool                        isAvail;                  /* Used for checking feature availability. */
    int                         deviceIndex;              /* Index of device used in following variables. */
    PYLON_WAITOBJECTS_HANDLE    wos;                      /* Wait objects. */
#ifdef GENAPIC_WIN_BUILD
    HANDLE                      hTimer;                   /* Grab timer. */
#else
    int                         fdTimer;                  /* Grab timer. */
#endif
    PYLON_WAITOBJECT_HANDLE     woTimer;                  /* Timer wait object. */

    /* These are camera specific variables */
    PYLON_DEVICE_HANDLE         hDev[NUM_DEVICES];        /* Handle for the pylon device. */
    PYLON_STREAMGRABBER_HANDLE  hGrabber[NUM_DEVICES];    /* Handle for the pylon stream grabber. */
    unsigned char              *buffers[NUM_DEVICES][NUM_BUFFERS];    /* Buffers used for grabbing. */
    PYLON_STREAMBUFFER_HANDLE   bufHandles[NUM_DEVICES][NUM_BUFFERS]; /* Handles for the buffers. */

    /* Before using any pylon methods, the pylon runtime must be initialized. */
    PylonInitialize();

    /* Enumerate all devices. You must call
    PylonEnumerateDevices() before creating a device. */
    res = PylonEnumerateDevices( &numDevicesAvail );
    CHECK(res);
    if ( numDevicesAvail < NUM_DEVICES)
    {
        fprintf( stderr, "Not enough devices found. Found %u devices. At least %i devices needed to run this sample.\n", (unsigned int) numDevicesAvail, NUM_DEVICES );
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_SUCCESS);
    }

    /* Create wait objects (must be done outside of the loop). */
    res = PylonWaitObjectsCreate(&wos);
    CHECK(res);

    /* In this sample, we want to grab for a given amount of time, then stop. */
#ifdef GENAPIC_WIN_BUILD
    /* Create a Windows timer, wrap it in a pylon C wait object, and add it to
       the wait object set. */
    hTimer = CreateWaitableTimer(NULL, TRUE, NULL);
    if (hTimer == NULL)
    {
        fprintf( stderr, "CreateWaitableTimer() failed.\n" );
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_FAILURE);
    }
    res = PylonWaitObjectFromW32(hTimer, 0, &woTimer);
    CHECK(res);
#else
    /* Create a Linux timer, wrap it in a pylon C wait object, and add it to
       the wait object set. */
    fdTimer = timerfd_create(CLOCK_MONOTONIC, 0);
    if (fdTimer == -1)
    {
        fprintf( stderr, "timerfd_create() failed. %s\n", strerror(errno) );
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_FAILURE);
    }
    res = PylonWaitObjectFromFd(fdTimer, &woTimer);
    CHECK(res);
#endif

    res = PylonWaitObjectsAdd(wos, woTimer, NULL);
    CHECK(res);

    /* Open cameras and set parameters. */
    for (deviceIndex = 0; deviceIndex < NUM_DEVICES; ++deviceIndex)
    {
        PylonDeviceInfo_t di;

        /* Get a handle for the device. */
        res = PylonCreateDeviceByIndex( deviceIndex, &hDev[deviceIndex] );
        CHECK(res);

        /* Before using the device, it must be opened. Open it for setting
           parameters and for grabbing images. */
        res = PylonDeviceOpen( hDev[deviceIndex], PYLONC_ACCESS_MODE_CONTROL | PYLONC_ACCESS_MODE_STREAM );
        CHECK(res);

        /* Print out the name of the camera we are using. */
        {
            char buf[256];
            size_t siz = sizeof(buf);
            _Bool isReadable;

            isReadable = PylonDeviceFeatureIsReadable(hDev[deviceIndex], "DeviceModelName");
            if ( isReadable )
            {
                res = PylonDeviceFeatureToString( hDev[deviceIndex], "DeviceModelName", buf, &siz );
                CHECK(res);
                printf("Using camera '%s'\n", buf);
            }
        }

        /* Set the pixel format to Mono8, where gray values will be output as 8 bit values for each pixel. */
        /* ... First check to see if the device supports the Mono8 format. */
        isAvail = PylonDeviceFeatureIsAvailable(hDev[deviceIndex], "EnumEntry_PixelFormat_Mono8");
        if ( ! isAvail )
        {
            /* Feature is not available. */
            fprintf(stderr, "Device doesn't support the Mono8 pixel format");
            PylonTerminate();
            pressEnterToExit();
            exit (EXIT_FAILURE);
        }

        /* ... Set the pixel format to Mono8. */
        res = PylonDeviceFeatureFromString( hDev[deviceIndex], "PixelFormat", "Mono8" );
        CHECK(res);

        /* Disable acquisition start trigger if available. */
        isAvail = PylonDeviceFeatureIsAvailable( hDev[deviceIndex], "EnumEntry_TriggerSelector_AcquisitionStart");
        if (isAvail)
        {
            res = PylonDeviceFeatureFromString( hDev[deviceIndex], "TriggerSelector", "AcquisitionStart");
            CHECK(res);
            res = PylonDeviceFeatureFromString( hDev[deviceIndex], "TriggerMode", "Off");
            CHECK(res);
        }

        /* Disable frame burst start trigger if available. */
        isAvail = PylonDeviceFeatureIsAvailable( hDev[deviceIndex], "EnumEntry_TriggerSelector_FrameBurstStart");
        if (isAvail)
        {
            res = PylonDeviceFeatureFromString( hDev[deviceIndex], "TriggerSelector", "FrameBurstStart");
            CHECK(res);
            res = PylonDeviceFeatureFromString( hDev[deviceIndex], "TriggerMode", "Off");
            CHECK(res);
        }

        /* Disable frame start trigger if available. */
        isAvail = PylonDeviceFeatureIsAvailable( hDev[deviceIndex], "EnumEntry_TriggerSelector_FrameStart");
        if (isAvail)
        {
            res = PylonDeviceFeatureFromString( hDev[deviceIndex], "TriggerSelector", "FrameStart");
            CHECK(res);
            res = PylonDeviceFeatureFromString( hDev[deviceIndex], "TriggerMode", "Off");
            CHECK(res);
        }

        /* We will use the Continuous frame mode, i.e., the camera delivers images continuously. */
        res = PylonDeviceFeatureFromString( hDev[deviceIndex], "AcquisitionMode", "Continuous" );
        CHECK(res);


        res = PylonDeviceGetDeviceInfo( hDev[deviceIndex], &di);
        CHECK(res);
        if (strcmp(di.DeviceClass, "BaslerGigE") == 0)
        {
            /* For GigE cameras, we recommend increasing the packet size for better
               performance. When the network adapter supports jumbo frames, set the packet
               size to a value > 1500, e.g., to 8192. In this sample, we only set the packet size
               to 1500.

               Also we set the Inter-Packet and the Frame Transmission delay
               so the switch can line up packets better.
            */

            res = PylonDeviceSetIntegerFeature( hDev[deviceIndex], "GevSCPSPacketSize", GIGE_PACKET_SIZE );
            CHECK(res);

            res = PylonDeviceSetIntegerFeature( hDev[deviceIndex], "GevSCPD", (GIGE_PACKET_SIZE + GIGE_PROTOCOL_OVERHEAD)*(NUM_DEVICES-1) );
            CHECK(res);

            res = PylonDeviceSetIntegerFeature( hDev[deviceIndex], "GevSCFTD", (GIGE_PACKET_SIZE + GIGE_PROTOCOL_OVERHEAD) * deviceIndex );
            CHECK(res);
        }
#ifdef GENAPIC_WIN_BUILD
        else if (strcmp(di.DeviceClass, "Basler1394") == 0)
        {
            /* For FireWire we just set the PacketSize node to limit the bandwidth we're using. */

            /* We first divide the available bandwidth (4915 for FW400, 9830 for FW800)
               by the number of devices we are using. */
            int64_t newPacketSize = 4915 / NUM_DEVICES;
            int64_t recommendedPacketSize = 0;

            /* Get the recommended packet size from the camera. */
            res = PylonDeviceGetIntegerFeature( hDev[deviceIndex], "RecommendedPacketSize", &recommendedPacketSize );
            CHECK(res);

            if (newPacketSize < recommendedPacketSize)
            {
                /* Get the increment value for the packet size.
                   We must make sure that the new value we're setting is dividable by the increment of that feature. */
                int64_t packetSizeInc = 0;
                res = PylonDeviceGetIntegerFeatureInc( hDev[deviceIndex], "PacketSize", &packetSizeInc);
                CHECK(res);

                /* Adjust the new packet size so is dividable by its increment. */
                newPacketSize -= newPacketSize % packetSizeInc;
            }
            else
            {
                /* The recommended packet size should always be valid. Accordingly, there will be no need to check against the increment. */
                newPacketSize = recommendedPacketSize;
            }


            /* Set the new packet size. */
            res = PylonDeviceSetIntegerFeature( hDev[deviceIndex], "PacketSize", newPacketSize);
            CHECK(res);

#if __STDC_VERSION__ >= 199901L
            printf("Using packetsize: %lld\n", newPacketSize);
#else
            printf("Using packetsize: %I64d\n", newPacketSize);
#endif
        }
#endif
    }


    /* Allocate and register buffers for grab. */
    for (deviceIndex = 0; deviceIndex < NUM_DEVICES; ++deviceIndex)
    {
        size_t i;
        PYLON_WAITOBJECT_HANDLE hWait;
        int32_t payloadSize;


        /* Determine the required size of the grab buffer. */
        res = PylonDeviceGetIntegerFeatureInt32( hDev[deviceIndex], "PayloadSize", &payloadSize );
        CHECK(res);


        /* Image grabbing is done using a stream grabber.
           A device may be able to provide different streams. A separate stream grabber must
           be used for each stream. In this sample, we create a stream grabber for the default
           stream, i.e., the first stream ( index == 0 ). */

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
        for ( i = 0; i < NUM_BUFFERS; ++i )
        {
            buffers[deviceIndex][i] = (unsigned char *) malloc ( payloadSize );
            if ( NULL == buffers[deviceIndex][i] )
            {
                fprintf( stderr, "Out of memory.\n" );
                PylonTerminate();
                pressEnterToExit();
                exit(EXIT_FAILURE);
            }
        }

        /* We must tell the stream grabber the number and size of the buffers we are using. */
        /* .. We will not use more than NUM_BUFFERS for grabbing. */
        res = PylonStreamGrabberSetMaxNumBuffer( hGrabber[deviceIndex], NUM_BUFFERS );
        CHECK(res);
        /* .. We will not use buffers bigger than payloadSize bytes. */
        res = PylonStreamGrabberSetMaxBufferSize( hGrabber[deviceIndex], payloadSize );
        CHECK(res);


        /* Allocate the resources required for grabbing. After this, critical parameters
           that impact the payload size must not be changed until FinishGrab() is called. */
        res = PylonStreamGrabberPrepareGrab( hGrabber[deviceIndex] );
        CHECK(res);


        /* Before using the buffers for grabbing, they must be registered at
           the stream grabber. For each registered buffer, a buffer handle
           is returned. After registering, these handles are used instead of the
           raw pointers. */
        for ( i = 0; i < NUM_BUFFERS; ++i )
        {
            res = PylonStreamGrabberRegisterBuffer( hGrabber[deviceIndex], buffers[deviceIndex][i], payloadSize,  &bufHandles[deviceIndex][i] );
            CHECK(res);
        }

        /* Feed the buffers into the stream grabber's input queue. For each buffer, the API
           allows passing in a pointer to additional context information. This pointer
           will be returned unchanged when the grab is finished. In our example, we use the index of the
           buffer as context information. */
        for ( i = 0; i < NUM_BUFFERS; ++i )
        {
            res = PylonStreamGrabberQueueBuffer( hGrabber[deviceIndex], bufHandles[deviceIndex][i], (void *) i );
            CHECK(res);
        }
    }


    /* The stream grabber is now prepared. As soon the camera starts to acquire images,
       the image data will be grabbed into the provided buffers.  */
    for (deviceIndex = 0; deviceIndex < NUM_DEVICES; ++deviceIndex)
    {
        /* Let the camera acquire images. */
        res = PylonDeviceExecuteCommandFeature( hDev[deviceIndex], "AcquisitionStart");
        /* do not call CHECK() here! Instead exit the loop */
        if (res != GENAPI_E_OK)
        {
            break;
        }
    }


    /* Only start the grab loop if all cameras have been "started" */
    if (res == GENAPI_E_OK)
    {
        unsigned int nGrabs = 0;

        /* Set the timer to 5 s and start it. */
#ifdef GENAPIC_WIN_BUILD
        LARGE_INTEGER intv;

        intv.QuadPart = -50000000I64;
        if (!SetWaitableTimer(hTimer, &intv, 0, NULL, NULL, FALSE))
        {
            fprintf( stderr, "SetWaitableTimer() failed.\n" );
            PylonTerminate();
            pressEnterToExit();
            exit(EXIT_FAILURE);
        }
#else
        struct itimerspec timer_value;

        timer_value.it_interval.tv_sec = 0;
        timer_value.it_interval.tv_nsec = 0;
        timer_value.it_value.tv_sec = 5;
        timer_value.it_value.tv_nsec = 0;
        if (timerfd_settime(fdTimer, 0, &timer_value, NULL) == -1)
        {
            fprintf( stderr, "timerfd_settime() failed. %s\n", strerror(errno) );
            PylonTerminate();
            pressEnterToExit();
            exit(EXIT_FAILURE);
        }
#endif

        /* Grab until the timer expires. */
        for (;;)
        {
            _Bool isReady;
            size_t woidx;
            unsigned char min, max;
            PylonGrabResult_t grabResult;

            /* Wait for the next buffer to be filled. Wait up to 1000 ms. */
            res = PylonWaitObjectsWaitForAny(wos, 1000, &woidx, &isReady );
            CHECK(res);
            if ( !isReady )
            {
                /* Timeout occurred. */
                fputs("Grab timeout occurred.\n", stderr);
                break; /* Stop grabbing. */
            }

            /* If the timer has expired, exit the grab loop */
            if (woidx == 0) {
                fputs("Grabbing completed successfully.\n", stderr);
                break;  /* timer expired */
            }

            /* Account for the timer. */
            --woidx;

            /* Retrieve the grab result. */
            res = PylonStreamGrabberRetrieveResult( hGrabber[woidx], &grabResult, &isReady );
            CHECK(res);
            if ( !isReady )
            {
                /* Oops. No grab result available? We should never have reached this point.
                   Since the wait operation above returned without a timeout, a grab result
                   should be available. */
                fprintf(stderr, "Failed to retrieve a grab result\n");
                break;
            }

            /* Check to see if the image was grabbed successfully. */
            if ( grabResult.Status == Grabbed )
            {
                /* Success. Perform image processing. Since we passed more than one buffer
                   to the stream grabber, the remaining buffers are filled while
                   we do the image processing. The processed buffer won't be touched by
                   the stream grabber until we pass it back to the stream grabber. */

                /* Pointer to the buffer attached to the grab result
                   Get the buffer pointer from the result structure. Since we also got the buffer index,
                   we could alternatively use buffers[bufferIndex]. */
                unsigned char* buffer = (unsigned char*) grabResult.pBuffer;

                /* Perform processing. */
                getMinMax( buffer, grabResult.SizeX, grabResult.SizeY, &min, &max );
                printf("Grabbed frame #%2u from camera %2u into buffer %2ld. Min. val=%3u, Max. val=%3u\n",
                       nGrabs, (unsigned int) woidx, (long) grabResult.Context, min, max);

#ifdef GENAPIC_WIN_BUILD
                /* Display image */
                res = PylonImageWindowDisplayImageGrabResult(woidx, &grabResult);
                CHECK(res);
#endif
            }
            else if ( grabResult.Status == Failed )
            {
                fprintf( stderr,  "Frame %u wasn't grabbed successfully.  Error code = 0x%08X\n",
                    nGrabs, grabResult.ErrorCode );
            }

            /* Once finished with the processing, requeue the buffer to be filled again. */
            res = PylonStreamGrabberQueueBuffer( hGrabber[woidx], grabResult.hBuffer, grabResult.Context );
            CHECK(res);

            nGrabs++;
        }
    }


    /* Clean up. */

    /* Stop the image acquisition on the cameras. */
    for (deviceIndex = 0; deviceIndex < NUM_DEVICES; ++deviceIndex)
    {
        /* ... Stop the camera. */
        res = PylonDeviceExecuteCommandFeature(hDev[deviceIndex], "AcquisitionStop");
        CHECK(res);
    }

    /* Remove all wait objects from waitobjects. */
    res = PylonWaitObjectsRemoveAll(wos);
    CHECK(res);
    res = PylonWaitObjectDestroy(woTimer);
    CHECK(res);
    res = PylonWaitObjectsDestroy(wos);
    CHECK(res);


    for (deviceIndex = 0; deviceIndex < NUM_DEVICES; ++deviceIndex)
    {
        size_t i;
        _Bool rdy;
        PylonGrabResult_t grabResult;

        /* ... We must issue a cancel call to ensure that all pending buffers are put into the
           stream grabber's output queue. */
        res = PylonStreamGrabberCancelGrab( hGrabber[deviceIndex] );
        CHECK(res);

        /* ... The buffers can now be retrieved from the stream grabber. */
        do
        {
            res = PylonStreamGrabberRetrieveResult( hGrabber[deviceIndex], &grabResult, &rdy );
            CHECK(res);
        } while ( rdy );

        /* ... When all buffers are retrieved from the stream grabber, they can be deregistered.
           After deregistering the buffers, it is safe to free the memory. */

        for ( i = 0; i < NUM_BUFFERS; ++i )
        {
            res = PylonStreamGrabberDeregisterBuffer( hGrabber[deviceIndex], bufHandles[deviceIndex][i] );
            CHECK(res);
            free( buffers[deviceIndex][i] );
        }

        /* ... Release grabbing related resources. */
        res = PylonStreamGrabberFinishGrab( hGrabber[deviceIndex] );
        CHECK(res);

        /* After calling PylonStreamGrabberFinishGrab(), parameters that impact the payload size (e.g.,
           the AOI width and height parameters) are unlocked and can be modified again. */

        /* ... Close the stream grabber. */
        res = PylonStreamGrabberClose( hGrabber[deviceIndex] );
        CHECK(res);


        /* ... Close and release the pylon device. The stream grabber becomes invalid
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

#ifdef GENAPIC_LINUX_BUILD
    close(fdTimer);
#endif
    return EXIT_SUCCESS;
}

/* This method demonstrates how to retrieve the error message for the last failed
   function call. */
void printErrorAndExit( GENAPIC_RESULT errc )
{
    char *errMsg;
    size_t length;

    /* Retrieve the error message.
       ... First find out how big the buffer must be, */
    GenApiGetLastErrorMessage( NULL, &length );
    errMsg = (char*) alloca( length );
    /* ... and retrieve the message. */
    GenApiGetLastErrorMessage( errMsg, &length );

    fprintf( stderr, "%s (%#08x).\n", errMsg, (unsigned int) errc);

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

