/*
Basler GigE Vision, IEEE 1394, and Camera Link cameras can send event messages. For example, when a sensor
exposure has finished, the camera can send an end-of-exposure event to the PC. The event
can be received by the PC before the image data for the finished exposure has been completely
transferred. This sample illustrates the retrieving and processing of event messages.

Receiving events is very similar to grabbing images. An event grabber provides a wait object that
is signalled when an event message is available. When an event message is available, it can be
retrieved from the event grabber. In contrast to grabbing images, memory buffers for receiving
events need not be provided by the application. Memory buffers to store event messages are organized
by the event grabber itself.

The specific layout of event messages depends on the event type and the camera type. The pylon API
uses GenICam support for parsing event messages. This means that the message layout is described in the
camera's XML description file. A GenApi node map is created from the XML camera description file.
This node map contains node objects representing the elements of the XML file. Since the layout of event
messages is described in the camera description file, the information carried by the event messages is
exposed as nodes in the node map and can be accessed like "normal" camera parameters.


You can register callback functions that are fired when a parameter has been changed. To be
informed that a received event message contains a specific event, a callback must be registered for
the parameter(s) associated with the event.

These mechanisms are demonstrated with the end-of-exposure event. The event carries the following
information:
* ExposureEndEventFrameID: indicates the number of the image frame that has been exposed.
* ExposureEndEventTimestamp: indicates the moment when the event has been generated.
* ExposureEndEventStreamChannelIndex: indicates the number of the image data stream used to
transfer the exposed frame.
A callback for the ExposureEndEventFrameID will be registered as an indicator for the arrival
of an end-of-exposure event.

*/


#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>

#include <pylonc/PylonC.h>

#define CHECK( errc ) if ( GENAPI_E_OK != errc ) printErrorAndExit( errc )

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void);

/* This method demonstrates how to retrieve the error message for the last failed function call. */
void printErrorAndExit( GENAPIC_RESULT errc );

/* Calculates the minimum and maximum gray value. */
void getMinMax( const unsigned char* pImg, int32_t width, int32_t height,
               unsigned char* pMin, unsigned char* pMax);

/* The function to be fired when an end of exposure event has been received. */
void GENAPIC_CC endOfExposureCallback( NODE_HANDLE hNode );


#define NUM_GRABS 100          /* Number of images to grab. */
#define NUM_IMAGE_BUFFERS 5   /* Number of buffers used for grabbing. */
#define NUM_EVENT_BUFFERS 20  /* Number of buffers used for grabbing. */

int main(void)
{
    GENAPIC_RESULT              res;                      /* Return value of pylon methods. */
    size_t                      numDevices;               /* Number of available devices. */
    PYLON_DEVICE_HANDLE         hDev;                     /* Handle for the pylon device. */
    PYLON_STREAMGRABBER_HANDLE  hStreamGrabber;           /* Handle for the pylon stream grabber. */
    PYLON_EVENTGRABBER_HANDLE   hEventGrabber;            /* Handle for the event grabber used for receiving events. */
    PYLON_EVENTADAPTER_HANDLE   hEventAdapter;            /* Handle for the event adapter used for dispatching events. */
    PYLON_WAITOBJECT_HANDLE     hWaitStream;              /* Handle used for waiting for a grab to be finished. */
    PYLON_WAITOBJECT_HANDLE     hWaitEvent;               /* Handle used for waiting for an event message. */
    PYLON_WAITOBJECTS_HANDLE    hWaitObjects;             /* Container allowing waiting for multiple wait objects. */
    NODEMAP_HANDLE              hNodeMap;                 /* Handle for the node map containing the
                                                             camera parameters. */
    NODE_CALLBACK_HANDLE        hCallback;                /* Used for deregistering a callback function. */
    NODE_HANDLE                 hNode;                    /* Handle for a camera parameter. */
    int32_t                     payloadSize;              /* Size of an image in bytes. */
    unsigned char              *buffers[NUM_IMAGE_BUFFERS];     /* Buffers used for grabbing. */
    PYLON_STREAMBUFFER_HANDLE   bufHandles[NUM_IMAGE_BUFFERS];  /* Handles for the buffers. */
    PylonGrabResult_t           grabResult;               /* Stores the result of a grab operation. */
    int                         nGrabs;                   /* Counts the number of buffers grabbed. */
    size_t                      nStreams;                 /* The number of streams the device provides. */
    _Bool                       isAvail;                  /* Used for checking feature availability. */
    _Bool                       isReady;                  /* Used as an output parameter. */
    size_t                      i;                        /* Counter. */
    int32_t                     sfncVersionMajor;         /* The major number of the Standard Feature Naming Convention (SFNC)
                                                             version used by the camera device. */

    /* Before using any pylon methods, the pylon runtime must be initialized. */
    PylonInitialize();

    /* Enumerate all camera devices. You must call
    PylonEnumerateDevices() before creating a device. */
    res = PylonEnumerateDevices( &numDevices );
    CHECK(res);
    if ( 0 == numDevices )
    {
        fprintf( stderr, "No devices found.\n" );
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_FAILURE);
    }

    /* Get a handle for the first device found.  */
    res = PylonCreateDeviceByIndex( 0, &hDev );
    CHECK(res);

    /* Before using the device, it must be opened. Open it for settig
       parameters, for grabbing images, and for grabbing events. */
    res = PylonDeviceOpen( hDev, PYLONC_ACCESS_MODE_CONTROL | PYLONC_ACCESS_MODE_STREAM | PYLONC_ACCESS_MODE_EVENT );
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

    /* Set the pixel format to Mono8 if available, where gray values will be output as 8 bit values for each pixel. */
    isAvail = PylonDeviceFeatureIsAvailable(hDev, "EnumEntry_PixelFormat_Mono8");
    if (isAvail)
    {
        res = PylonDeviceFeatureFromString( hDev, "PixelFormat", "Mono8" );
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

    /* We will use the Continuous frame mode, i.e., the camera delivers
    images continuously. */
    res = PylonDeviceFeatureFromString( hDev, "AcquisitionMode", "Continuous" );
    CHECK(res);


    /* For GigE cameras, we recommend increasing the packet size for better
       performance. If the network adapter supports jumbo frames, set the packet
       size to a value > 1500, e.g., to 8192. In this sample, we only set the packet size
       to 1500. */
    /* ... Check first to see if the GigE camera packet size parameter is supported and if it is writable. */
    isAvail = PylonDeviceFeatureIsWritable(hDev, "GevSCPSPacketSize");
    if ( isAvail )
    {
        /* ... The device supports the packet size feature, set a value. */
        res = PylonDeviceSetIntegerFeature( hDev, "GevSCPSPacketSize", 1500 );
        CHECK(res);
    }

    isAvail = PylonDeviceFeatureIsWritable(hDev, "EventSelector");
    if ( !isAvail )
    {
        /* Feature is not available. */
        fprintf(stderr, "Device doesn't support events.\n");
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_FAILURE);
    }

    /* Determine the major number of the SFNC version used by the camera device. */
    if ( PylonDeviceGetIntegerFeatureInt32( hDev, "DeviceSFNCVersionMajor", &sfncVersionMajor ) != GENAPI_E_OK )
    {
        /* No SFNC version information is provided by the camera device. */
        sfncVersionMajor = 0;
    }

    /* Enable camera event reporting. */
    /* Select the end-of-exposure event reporting. */
    res = PylonDeviceFeatureFromString( hDev, "EventSelector", "ExposureEnd" );
    CHECK(res);
    /* Enable the event reporting.
    Select the enumeration entry name depending on the SFNC version used by the camera device.
    */
    if ( sfncVersionMajor >= 2 )
        res = PylonDeviceFeatureFromString( hDev, "EventNotification", "On" );
    else
        res = PylonDeviceFeatureFromString( hDev, "EventNotification", "GenICamEvent" );
    CHECK(res);

    /* Determine the required size of the grab buffer. */
    res = PylonDeviceGetIntegerFeatureInt32( hDev, "PayloadSize", &payloadSize );
    CHECK(res);

    /* Image grabbing is done using a stream grabber.
    A device may be able to provide different streams. A separate stream grabber must
    be used for each stream. In this sample, we create a stream grabber for the default
    stream, i.e., the first stream ( index == 0 ).
    */

    /* Get the number of streams supported by the device and the transport layer. */
    res = PylonDeviceGetNumStreamGrabberChannels( hDev, &nStreams );
    CHECK(res);
    if ( nStreams < 1 )
    {
        fprintf( stderr, "The transport layer doesn't support image streams\n");
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_FAILURE);
    }

    /* Create and open a stream grabber for the first channel. */
    res = PylonDeviceGetStreamGrabber( hDev, 0, &hStreamGrabber );
    CHECK(res);
    res = PylonStreamGrabberOpen( hStreamGrabber );
    CHECK(res);

    /* Get a handle for the stream grabber's wait object. The wait object
    allows waiting for buffers to be grabbed. */
    res = PylonStreamGrabberGetWaitObject( hStreamGrabber, &hWaitStream );
    CHECK(res);

    /* Allocate memory for grabbing.  */
    for ( i = 0; i < NUM_IMAGE_BUFFERS; ++i )
    {
        buffers[i] = (unsigned char*) malloc ( payloadSize );
        if ( NULL == buffers[i] )
        {
            fprintf( stderr, "Out of memory!\n" );
            PylonTerminate();
            pressEnterToExit();
            exit(EXIT_FAILURE);
        }
    }

    /* We must tell the stream grabber the number and size of the buffers
    we are using. */
    /* .. We will not use more than NUM_BUFFERS for grabbing. */
    res = PylonStreamGrabberSetMaxNumBuffer( hStreamGrabber, NUM_IMAGE_BUFFERS );
    CHECK(res);
    /* .. We will not use buffers bigger than payloadSize bytes. */
    res = PylonStreamGrabberSetMaxBufferSize( hStreamGrabber, payloadSize );
    CHECK(res);


    /*  Allocate the resources required for grabbing. After this, critical parameters
    that impact the payload size must not be changed until FinishGrab() is called. */
    res = PylonStreamGrabberPrepareGrab( hStreamGrabber );
    CHECK(res);


    /* Before using the buffers for grabbing, they must be registered at
    the stream grabber. For each registered buffer, a buffer handle
    is returned. After registering, these handles are used instead of the
    raw pointers. */
    for ( i = 0; i < NUM_IMAGE_BUFFERS; ++i )
    {
        res = PylonStreamGrabberRegisterBuffer( hStreamGrabber, buffers[i], payloadSize,  &bufHandles[i] );
        CHECK(res);
    }

    /* Feed the buffers into the stream grabber's input queue. For each buffer, the API
    allows passing in a pointer to additional context information. This pointer
    will be returned unchanged when the grab is finished. In our example, we use the index of the
    buffer as context information. */
    for ( i = 0; i < NUM_IMAGE_BUFFERS; ++i )
    {
        res = PylonStreamGrabberQueueBuffer( hStreamGrabber, bufHandles[i], (void*) i );
        CHECK(res);
    }

    /* The stream grabber is now prepared. As soon as the camera starts to acquire images,
    the image data will be grabbed into the provided buffers.  */


    /* Create and prepare an event grabber. */
    /* ... Get a handle for the event grabber. */
    res = PylonDeviceGetEventGrabber( hDev, &hEventGrabber );
    CHECK(res);
    if ( hEventGrabber == PYLONC_INVALID_HANDLE )
    {
        /* The transport layer doesn't support event grabbers. */
        fprintf(stderr, "No event grabber supported.\n");
        PylonTerminate();
        pressEnterToExit();
        return EXIT_FAILURE;
    }

    /* ... Tell the grabber how many buffers to use. */
    res = PylonEventGrabberSetNumBuffers( hEventGrabber, NUM_EVENT_BUFFERS );
    CHECK(res);

    /* ... Open the event grabber. */
    res = PylonEventGrabberOpen( hEventGrabber );  /* The event grabber is now ready
                                                   for receiving events. */
    CHECK(res);

    /* Retrieve the wait object that is associated with the event grabber. The event
    will be signaled when an event message has been received. */
    res = PylonEventGrabberGetWaitObject( hEventGrabber, &hWaitEvent );
    CHECK(res);

    /* For extracting the event data from an event message, an event adapter is used. */
    res = PylonDeviceCreateEventAdapter( hDev, &hEventAdapter );
    CHECK(res);
    if ( hEventAdapter == PYLONC_INVALID_HANDLE )
    {
        /* The transport layer doesn't support event grabbers. */
        fprintf(stderr, "No event adapter supported.\n");
        PylonTerminate();
        pressEnterToExit();
        return EXIT_FAILURE;
    }

    /* Register the callback function for ExposureEndEventFrameID parameter. */
    /*... Get the node map containing all parameters. */
    res = PylonDeviceGetNodeMap( hDev, &hNodeMap );
    CHECK(res);
    /* Get the ExposureEndEventFrameID parameter.
    Select the parameter name depending on the SFNC version used by the camera device.
    */
    if ( sfncVersionMajor >= 2 )
        res = GenApiNodeMapGetNode( hNodeMap, "EventExposureEndFrameID", &hNode );
    else
        res = GenApiNodeMapGetNode( hNodeMap, "ExposureEndEventFrameID", &hNode );
    CHECK(res);

    if ( GENAPIC_INVALID_HANDLE == hNode )
    {
        /* There is no ExposureEndEventFrameID parameter. */
        fprintf( stderr, "There is no ExposureEndEventFrameID or EventExposureEndFrameID parameter.\n");
        PylonTerminate();
        pressEnterToExit();
        return EXIT_FAILURE;
    }

    /* ... Register the callback function. */
    res = GenApiNodeRegisterCallback( hNode, endOfExposureCallback, &hCallback );
    CHECK(res);

    /* Put the wait objects into a container. */
    /* ... Create the container. */
    res = PylonWaitObjectsCreate( &hWaitObjects );
    CHECK(res);
    /* ... Add the wait objects' handles. */
    res = PylonWaitObjectsAddMany( hWaitObjects, 2, hWaitEvent, hWaitStream);
    CHECK(res);

    /* Let the camera acquire images. */
    res = PylonDeviceExecuteCommandFeature( hDev, "AcquisitionStart");
    CHECK(res);

    /* Grab NUM_GRABS images. */
    nGrabs = 0;                         /* Counts the number of images grabbed. */
    while ( nGrabs < NUM_GRABS )
    {
        size_t bufferIndex;              /* Index of the buffer. */
        size_t waitObjectIndex;          /* Index of the wait object that is signalled.*/
        unsigned char min, max;

        /* Wait for either an image buffer grabbed or an event received. Wait up to 1000 ms. */
        res = PylonWaitObjectsWaitForAny( hWaitObjects, 1000, &waitObjectIndex, &isReady );
        CHECK(res);
        if ( ! isReady )
        {
            /* Timeout occurred. */
            fprintf(stderr, "Timeout. Neither grabbed an image nor received an event.\n");
            break; /* Stop grabbing. */
        }

        if ( 0 == waitObjectIndex )
        {
            PylonEventResult_t eventMsg;
            /* hWaitEvent has been signalled. At least one event message is available. Retrieve it. */
            res = PylonEventGrabberRetrieveEvent( hEventGrabber, &eventMsg, &isReady );
            CHECK(res);
            if ( ! isReady )
            {
                /* Oops. No event message available? We should never have reached this point.
                Since the wait operation above returned without a timeout, an event message
                should be available. */
                fprintf(stderr, "Failed to retrieve an event\n");
                break;
            }
            /* Check to see if the event was successfully received. */
            if ( 0 == eventMsg.ErrorCode )
            {
                /* Successfully received an event message. */
                /* Pass the event message to the event adapter. The event adapter will
                update the parameters related to events and will fire the callbacks
                registered to event related parameters. */
                res = PylonEventAdapterDeliverMessage( hEventAdapter, &eventMsg );
                CHECK(res);
            }
            else
            {
                fprintf(stderr, "Error when receiving an event: 0x%08x\n", eventMsg.ErrorCode );
            }
        }
        else if ( 1 == waitObjectIndex )
        {
            /* hWaitStream has been signalled. The result of at least one grab
            operation is available. Retrieve it. */
            res = PylonStreamGrabberRetrieveResult( hStreamGrabber, &grabResult, &isReady );
            CHECK(res);
            if ( ! isReady )
            {
                /* Oops. No grab result available? We should never have reached this point.
                Since the wait operation above returned without a timeout, a grab result
                should be available. */
                fprintf(stderr, "Failed to retrieve a grab result\n");
                break;
            }

            nGrabs++;

            /* Get the buffer index from the context information. */
            bufferIndex = (size_t) grabResult.Context;

            /* Check to see if the image was grabbed successfully. */
            if ( grabResult.Status == Grabbed )
            {
                /*  Success. Perform image processing. Since we passed more than one buffer
                to the stream grabber, the remaining buffers are filled while
                we do the image processing. The processed buffer won't be touched by
                the stream grabber until we pass it back to the stream grabber. */

                unsigned char* buffer;        /* Pointer to the buffer attached to the grab result. */

                /* Get the buffer pointer from the result structure. Since we also got the buffer index,
                we could alternatively use buffers[bufferIndex]. */
                buffer = (unsigned char*) grabResult.pBuffer;


                getMinMax( buffer, grabResult.SizeX, grabResult.SizeY, &min, &max );
                printf("Grabbed frame #%2d into buffer %2d. Min. gray value = %3u, Max. gray value = %3u\n",
                    nGrabs, (int)bufferIndex, min, max);
            }
            else if ( grabResult.Status == Failed )
            {
                fprintf( stderr,  "Frame %d wasn't grabbed successfully.  Error code = 0x%08X\n",
                    nGrabs, grabResult.ErrorCode );
            }

            /* Once finished with the processing, requeue the buffer to be filled again. */
            res = PylonStreamGrabberQueueBuffer( hStreamGrabber, grabResult.hBuffer, (void*) bufferIndex );
            CHECK(res);
        }
    }

    /* Clean up. */

    /*  ... Stop the camera. */
    res = PylonDeviceExecuteCommandFeature( hDev, "AcquisitionStop");
    CHECK(res);

    /* ... Switch-off the events. */
    res = PylonDeviceFeatureFromString( hDev, "EventSelector", "ExposureEnd" );
    CHECK(res);
    res = PylonDeviceFeatureFromString( hDev, "EventNotification", "Off" );
    CHECK(res);


    /* ... We must issue a cancel call to ensure that all pending buffers are put into the
    stream grabber's output queue. */
    res = PylonStreamGrabberCancelGrab( hStreamGrabber );
    CHECK(res);

    /* ... The buffers can now be retrieved from the stream grabber. */
    do
    {
        res = PylonStreamGrabberRetrieveResult( hStreamGrabber, &grabResult, &isReady );
        CHECK(res);
    } while ( isReady );

    /* ... When all buffers are retrieved from the stream grabber, they can be deregistered.
    After deregistering the buffers, it is safe to free the memory. */

    for ( i = 0; i < NUM_IMAGE_BUFFERS; ++i )
    {
        res = PylonStreamGrabberDeregisterBuffer( hStreamGrabber, bufHandles[i] );
        CHECK(res);
        free( buffers[i] );
    }

    /* ... Release grabbing related resources. */
    res = PylonStreamGrabberFinishGrab( hStreamGrabber );
    CHECK(res);

    /* After calling PylonStreamGrabberFinishGrab(), parameters that impact the payload size (e.g.,
    the AOI width and height parameters) are unlocked and can be modified again. */

    /* ... Close the stream grabber. */
    res = PylonStreamGrabberClose( hStreamGrabber );
    CHECK(res);

    /* ... Deregister the callback. */
    res = GenApiNodeDeregisterCallback( hNode, hCallback );
    CHECK(res);

    /* ... Close the event grabber.*/
    res = PylonEventGrabberClose( hEventGrabber );
    CHECK(res);

    /* ... Release the event adapter. */
    res = PylonDeviceDestroyEventAdapter( hDev, hEventAdapter );
    CHECK(res);

    /* ... Release the wait object container. */
    res = PylonWaitObjectsDestroy( hWaitObjects );
    CHECK(res);


    /* ... Close and release the pylon device. The stream grabber becomes invalid
    after closing the pylon device. Don't call stream grabber related methods after
    closing or releasing the device. */
    res = PylonDeviceClose( hDev );
    CHECK(res);
    res = PylonDestroyDevice ( hDev );
    CHECK(res);


    /* ... Shut down the pylon runtime system. Don't call any pylon method after
    calling PylonTerminate(). */
    PylonTerminate();
    pressEnterToExit();

    return EXIT_SUCCESS;
}

/* This function demonstrates how to retrieve the error message for the last failed
   function call. */
void printErrorAndExit( GENAPIC_RESULT errc )
{
    char *errMsg;
    size_t length;

    /* Retrieve the error message.
    ... First find out how big the buffer must be, */
    GenApiGetLastErrorMessage( NULL, &length );
    errMsg = (char*) malloc( length );
    /* ... and retrieve the message. */
    GenApiGetLastErrorMessage( errMsg, &length );

    fprintf( stderr, "%s (%#08x).\n", errMsg, (unsigned int) errc);
    free( errMsg);

    /* Retrieve more details about the error
    ... First find out how big the buffer must be, */
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



/* Simple "image processing" function returning the minumum and maximum gray
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

/* Callback will be fired when an event message contains an end-of-exposure event. */
void GENAPIC_CC endOfExposureCallback( NODE_HANDLE hNode )
{
    int64_t frame;
    GENAPIC_RESULT res;
    res = GenApiIntegerGetValue( hNode, &frame );
    CHECK(res);

#if __STDC_VERSION__ >= 199901L || defined(__GNUC__)
    printf("Got end-of-exposure event. Frame number: %lld\n", (long long) frame );
#else
    printf("Got end-of-exposure event. Frame number: %I64d\n", frame );
#endif
}

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void)
{
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
}

