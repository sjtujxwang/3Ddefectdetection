// CameraEvents.cpp
//
/*
    Important: This sample shows how to migrate existing Low Level API code
    in order to be able to use USB3 Vision compliant cameras. Before migrating code,
    Basler strongly recommends reading the "Migrating from Previous Versions" topic in the pylon C++ API documentation
    that gets installed with pylon.

    The sample code below shows how to receive and interpret event messages sent by the camera.
    The usage of the Exposure End event is shown.
*/

// Include files to use the pylon API
#include <pylon/PylonIncludes.h>
using namespace Pylon;


#if defined( USE_1394 )
// Settings to use Basler 1394 cameras
#include <pylon/1394/Basler1394Camera.h>
typedef Pylon::CBasler1394Camera Camera_t;
using namespace Basler_IIDC1394CameraParams;
using namespace Basler_IIDC1394StreamParams;
#elif defined ( USE_GIGE )
// Settings to use Basler GigE cameras
#include <pylon/gige/BaslerGigECamera.h>
typedef Pylon::CBaslerGigECamera Camera_t;
using namespace Basler_GigECameraParams;
using namespace Basler_GigEStreamParams;
using namespace Basler_GigETLParams;
#elif defined ( USE_USB )
// Settings to use Basler USB cameras
#include <pylon/usb/BaslerUsbCamera.h>
typedef Pylon::CBaslerUsbCamera Camera_t;
using namespace Basler_UsbCameraParams;
using namespace Basler_UsbStreamParams;
#else
#error Camera type is not specified. For example, define USE_GIGE for using GigE cameras
#endif


// Namespace for using cout
using namespace std;

// This function can be used to wait for user input at the end of the sample program.
void pressEnterToExit()
{
    //Comment the following two lines to disable wait on exit here.
    cerr << endl << "Press enter to exit." << endl;
    while( cin.get() != '\n');
}

// Simple buffer class
class CGrabBuffer
{
    public:
        CGrabBuffer(const size_t ImageSize);
        ~CGrabBuffer();
        uint8_t* GetBufferPointer(void) { return pBuffer; }
        StreamBufferHandle GetBufferHandle(void) { return m_hBuffer; }
        void SetBufferHandle(StreamBufferHandle hBuffer) { m_hBuffer = hBuffer; };

    protected:
        uint8_t *pBuffer;
        StreamBufferHandle m_hBuffer;
};

// Constructor allocates the image buffer
CGrabBuffer::CGrabBuffer(const size_t ImageSize):
        pBuffer(NULL)
{
    pBuffer = new uint8_t[ ImageSize ];
    if (NULL == pBuffer)
    {
        throw GENERIC_EXCEPTION("Not enough memory to allocate image buffer");
    }
}

// Freeing the memory
CGrabBuffer::~CGrabBuffer()
{
    if (NULL != pBuffer)
        delete[] pBuffer;
}

// Number of buffers used for grabbing images and events
const uint32_t c_nImageBuffers = 10;
const uint32_t c_nEventBuffers = 20;

// Number of images to be grabbed
const uint32_t c_ImagesToGrab = 20;

// Member function of this class will be registered as callback.
struct CallbackTarget
{
    CallbackTarget(Camera_t& Camera)
            : m_Camera(Camera)
    { }

    // Will be fired when an Exposure End event occurs
    void EndOfExposureCallback(GenApi::INode* pNode)
    {
        try
        {
            cout << "The message contains an Exposure End event." << endl;
#if defined( USE_USB )
            cout << "Timestamp: " << m_Camera.EventExposureEndTimestamp.GetValue() << " Frame number: " << m_Camera.EventExposureEndFrameID.GetValue() << endl;
#else
            cout << "Timestamp: " << m_Camera.ExposureEndEventTimestamp.GetValue() << " Frame number: " << m_Camera.ExposureEndEventFrameID.GetValue() << endl;
#endif
        }
        catch (const GenericException& e)
        {
            cerr << "Failed to get event information. Exception occurred:"
            << e.GetDescription() << endl;
        }
    }

    // Will be fired when a frame start overtrigger event occurs.
    void FrameStartOvertriggerCallback(GenApi::INode* pNode)
    {
        try
        {
            cout << "The message contains a FrameStart Overtrigger event." << endl;
#if defined( USE_USB )
            cout << "Timestamp: " << m_Camera.EventFrameStartOvertriggerTimestamp.GetValue() << endl;
#else
            cout << "Timestamp: " << m_Camera.FrameStartOvertriggerEventTimestamp.GetValue() << endl;
#endif
        }
        catch (const GenericException& e)
        {
            cerr << "Failed to get event information. Exception occurred:"
            << e.GetDescription() << endl;
        }
    }

    Camera_t& m_Camera;
};


int main(int argc, char* argv[])
{
    // Automagically call PylonInitialize and PylonTerminate to ensure that the pylon runtime
    // system is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        //
        // Create a camera object
        //

        // Create the transport layer object needed to enumerate or
        // create a camera object of Camera_t::DeviceClass() type.
        ITransportLayer *pTl = CTlFactory::GetInstance().CreateTl(Camera_t::DeviceClass());
        if (! pTl)
        {
            cerr << "Failed to create transport layer!" << endl;
            pressEnterToExit();
            return 1;
        }

        // Discover all attached devices
        DeviceInfoList_t devices;
        if (0 == pTl->EnumerateDevices(devices))
        {
            cerr << "No camera present!" << endl;
            pressEnterToExit();
            return 1;
        }

        // Create the camera object of the first available camera
        Camera_t Camera(pTl->CreateDevice(devices[ 0 ]));

        //
        // Open the camera
        //
        Camera.Open(Exclusive | Stream | Control | Event);

        //
        // Check if the device supports events
        //
        if (! GenApi::IsAvailable(Camera.EventSelector))
        {
            cerr << "Device doesn't support events" << endl;
            Camera.Close();
            pressEnterToExit();
            return 1;
        }

        //
        // Get and prepare the stream grabber
        //

        // Determine the image buffer size
        const int bufferSize = (int) Camera.PayloadSize.GetValue();

        // Get the stream grabber object for the first stream channel
        Camera_t::StreamGrabber_t StreamGrabber(Camera.GetStreamGrabber(0));

        // Open the grabber
        StreamGrabber.Open();


        //
        // Get and prepare the event grabber
        //

        // Create the event grabber
        Camera_t::EventGrabber_t EventGrabber(Camera.GetEventGrabber());

        // Parameterize and open it
        EventGrabber.NumBuffer.SetValue(c_nEventBuffers);    // must be set before open is called!!
#if USE_GIGE
        // Enable resending of event messages when lost messages are detected:
        // Loss of messages is detected by sending acknowledges for every event message.
        // When the camera doesn't receive the acknowledge, it will resend the message up to
        // 'RetryCount' times.
        EventGrabber.RetryCount = 3;
#endif
        EventGrabber.Open();

        //
        // Create the event adapter used for parsing the event messages
        //

        Pylon::IEventAdapter *pEventAdapter = Camera.CreateEventAdapter();
        if (! pEventAdapter)
        {
            cerr << "Failed to create an event adapter" << endl;
            pressEnterToExit();
            return 1;
        }

        //
        // Set up the camera for grabbing images continuously
        //

        Camera.PixelFormat.SetValue(PixelFormat_Mono8);


        // Check the available camera trigger mode(s) to select the appropriate one: acquisition start trigger mode (used by previous cameras;
        // do not confuse with acquisition start command) or frame start trigger mode (equivalent to previous acquisition start trigger mode).
        bool frameStartAvailable = false;
#ifndef USE_USB
        bool acquisitionStartAvailable = false;
#endif
        {
            // Frame start trigger mode available?
            GenApi::IEnumEntry* frameStart = Camera.TriggerSelector.GetEntry(TriggerSelector_FrameStart);
            frameStartAvailable = frameStart && GenApi::IsAvailable(frameStart);

#ifndef USE_USB
            // Acquisition start trigger mode available?
            GenApi::IEnumEntry* acquisitionStart = Camera.TriggerSelector.GetEntry(TriggerSelector_AcquisitionStart);
            acquisitionStartAvailable = acquisitionStart && GenApi::IsAvailable(acquisitionStart);
#endif
        }

        // Preselect the trigger mode for image acquisition.
        TriggerSelectorEnums triggerSelectorValue = TriggerSelector_FrameStart;

#ifndef USE_USB
        // Check to see if the camera implements the acquisition start trigger mode only.
        if ( acquisitionStartAvailable && !frameStartAvailable)
        {
            // Camera uses the acquisition start trigger as the only trigger mode.
            Camera.TriggerSelector.SetValue(TriggerSelector_AcquisitionStart);
            Camera.TriggerMode.SetValue(TriggerMode_On);
            triggerSelectorValue = TriggerSelector_AcquisitionStart;
        }
        else
        {
            // Camera may have the acquisition start trigger mode and the frame start trigger mode implemented.
            // In this case, the acquisition trigger mode must be switched off.
            if ( acquisitionStartAvailable )
            {
                Camera.TriggerSelector.SetValue(TriggerSelector_AcquisitionStart);
                Camera.TriggerMode.SetValue(TriggerMode_Off);
            }
#endif
            // To trigger each single frame by software or external hardware trigger: Enable the frame start trigger mode.
            assert( frameStartAvailable); //Frame start trigger mode must be available here.
            Camera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
            Camera.TriggerMode.SetValue(TriggerMode_On);
#ifndef USE_USB
        }
#endif

        // Note: the trigger selector must be set to the appropriate trigger mode
        // before setting the trigger source or issuing software triggers.
        // Frame start trigger mode for newer cameras, acquisition start trigger mode for older camera models.
        Camera.TriggerSelector.SetValue( triggerSelectorValue );

        //The trigger source must be set to 'Software'.
        Camera.TriggerSource.SetValue( TriggerSource_Software);

        //Set acquisition mode
        Camera.AcquisitionMode.SetValue(AcquisitionMode_Continuous);

        //Set exposure settings
#if defined( USE_USB )
            Camera.ExposureMode.SetValue(ExposureMode_Timed);
            Camera.ExposureTime.SetValue(70);
#else
            Camera.ExposureMode.SetValue(ExposureMode_Timed);
            Camera.ExposureTimeRaw.SetValue(70);
#endif

        // Set up the stream grabber
        StreamGrabber.MaxBufferSize.SetValue(bufferSize);
        StreamGrabber.MaxNumBuffer.SetValue(c_nImageBuffers);
        StreamGrabber.PrepareGrab();

        //
        // Enable the end of exposure event and the frame start overtrigger event.
        //

        Camera.EventSelector.SetValue(EventSelector_ExposureEnd);    // Select the event
#if defined( USE_USB )
        Camera.EventNotification.SetValue(EventNotification_On); // Enable it
#else
        Camera.EventNotification.SetValue(EventNotification_GenICamEvent);   // Enable it
#endif

        if ( IsAvailable(Camera.EventSelector.GetEntry(EventSelector_FrameStartOvertrigger)))
        {
            Camera.EventSelector.SetValue(EventSelector_FrameStartOvertrigger); // Select the event
#if defined( USE_USB )
            Camera.EventNotification.SetValue(EventNotification_On); // Enable it
#else
            Camera.EventNotification.SetValue(EventNotification_GenICamEvent);   // Enable it
#endif
        }

        //
        // Register a callback for the end of exposure event.
        //

        CallbackTarget callbackTarget(Camera);

        // Register the callback for the ExposureEndEventTimestamp node.
#if defined( USE_USB )
        GenApi::CallbackHandleType hCb = GenApi::Register(
                                             Camera.EventExposureEndTimestamp.GetNode(),
                                             callbackTarget,
                                             &CallbackTarget::EndOfExposureCallback);
#else
        GenApi::CallbackHandleType hCb = GenApi::Register(
                                             Camera.ExposureEndEventTimestamp.GetNode(),
                                             callbackTarget,
                                             &CallbackTarget::EndOfExposureCallback);
#endif

        // Register the callback for the FrameStart Overtrigger Event node.
#if defined( USE_USB )
        GenApi::CallbackHandleType hCb2 = GenApi::Register(
                                             Camera.EventFrameStartOvertriggerTimestamp.GetNode(),
                                             callbackTarget,
                                             &CallbackTarget::FrameStartOvertriggerCallback);
#else
        GenApi::CallbackHandleType hCb2 = 0;
        if ( IsAvailable(Camera.EventSelector.GetEntry(EventSelector_FrameStartOvertrigger)))
        {
            hCb2 = GenApi::Register(
                     Camera.FrameStartOvertriggerEventTimestamp.GetNode(),
                     callbackTarget,
                     &CallbackTarget::FrameStartOvertriggerCallback);
        }
#endif

        //
        // Create and register the image buffers.
        //
        std::vector<CGrabBuffer*> BufferList;
        for (uint32_t i = 0; i < c_nImageBuffers; ++i)
        {
            CGrabBuffer *pGrabBuffer = new CGrabBuffer(bufferSize);
            pGrabBuffer->SetBufferHandle(StreamGrabber.RegisterBuffer(
                                             pGrabBuffer->GetBufferPointer(), bufferSize));

            // Put the buffer into the grab queue for grabbing.
            StreamGrabber.QueueBuffer(pGrabBuffer->GetBufferHandle(), NULL);

            // Put the grab buffer object into the buffer list.
            BufferList.push_back(pGrabBuffer);
        }

        // Get the wait objects of the event grabber and the stream grabber.
        // Put the objects into a container to be able to wait for images and events
        // simultaneously.
        WaitObjects waitset;
        waitset.Add(EventGrabber.GetWaitObject());
        waitset.Add(StreamGrabber.GetWaitObject());

        // Let the camera acquire images continuously ( Acquisiton mode equals
        // Continuous! ).
        Camera.AcquisitionStart.Execute();

        // Issue the initial software trigger to the camera to get the first image (then see line 395).
        Camera.TriggerSoftware.Execute();

        //
        // Loop until c_ImagesToGrab images are grabbed.
        // Incoming events and images are processed.
        //

        uint32_t count = 0;
        while (count < c_ImagesToGrab)
        {
            unsigned int i;

            // Wait for an image or an event to occur (5 sec timeout).
            if (waitset.WaitForAny(5000, &i))
            {
                // Got event or image.
                switch (i)
                {
                    case 0: // Event available, process it
                    {
                        // Retrieve the event result.
                        EventResult EvResult;
                        if (EventGrabber.RetrieveEvent(EvResult))
                        {
                            if (EvResult.Succeeded())
                            {
                                cout << "Successfully got an event message!" << endl;
                                // To figure out the content of the event message, pass it to the event adapter.
                                // DeliverMessage will fire the registered callback when the buffer contains
                                // an Exposure End event.
                                pEventAdapter->DeliverMessage(EvResult.Buffer, sizeof EvResult.Buffer);
                            }
                            else
                            {
                                cerr << "Error retrieving event:" << EvResult.ErrorDescription() << endl;
                            }
                        }
                    }
                    break;

                    case 1: // Image available, process it
                    {
                        GrabResult GrResult;
                        if (StreamGrabber.RetrieveResult(GrResult))
                        {
                            if (GrResult.Succeeded())
                            {
                                count++;
                                // Grabbing was successful, process image
                                cout << "Grabbed image."  << endl;

                                // We have our image, so trigger the camera for the next one.
                                // Note: we now send two triggers to invoke an overtrigger situation.
                                Camera.TriggerSoftware.Execute();
                                Camera.TriggerSoftware.Execute();

                            }
                            else if (Failed == GrResult.Status())
                            {
                                // Error handling
                                cerr << "Failed to grab image!" << endl;
                                cerr << "Error code : 0x" << hex
                                << GrResult.GetErrorCode() << endl;
                                cerr << "Error description : "
                                << GrResult.GetErrorDescription() << endl;
                            }

                            // Reuse the buffer for grabbing further images.
                            if (count <= c_ImagesToGrab - c_nImageBuffers)
                                StreamGrabber.QueueBuffer(GrResult.Handle(), NULL);
                        }
                    }
                    break;
                }  // Switch
            }  // If WaitForAny
            else
            {
                // Timeout
                cerr << "Timeout occurred!" << endl;

                //
                // Get the pending image buffers back (you are not allowed to deregister
                // buffers when they are still queued).
                //

                // Release pending buffers.
                StreamGrabber.CancelGrab();

                // Get all buffers back.
                for (GrabResult r; StreamGrabber.RetrieveResult(r););

                // Cancel loop.
                break;
            }
        } // While

        // Stop image acquisition.
        Camera.AcquisitionStop.Execute();

        //
        // Cleanup
        //

        // Cleanup of image buffers.
        // You must deregister a buffer before freeing the memory.
        for (std::vector<CGrabBuffer*>::iterator it = BufferList.begin(); it != BufferList.end(); it++)
        {
            // Deregister buffer.
            StreamGrabber.DeregisterBuffer((*it)->GetBufferHandle());
            // Delete buffer.
            delete *it;
        }

        // Cleanup of stream grabber
        StreamGrabber.FinishGrab();
        StreamGrabber.Close();


        // Disable sending of Exposure End events.
        Camera.EventSelector = EventSelector_ExposureEnd;
        Camera.EventNotification.SetValue(EventNotification_Off);

        // Cleanup of event grabber and event adapter.
        // Deregister the callback.
#if defined ( USE_USB )
        Camera.EventExposureEndTimestamp.GetNode()->DeregisterCallback(hCb);
#else
        Camera.ExposureEndEventTimestamp.GetNode()->DeregisterCallback(hCb);
#endif


        if ( IsAvailable(Camera.EventSelector.GetEntry(EventSelector_FrameStartOvertrigger)))
        {
            // Disable sending of frame start overtrigger events.
            Camera.EventSelector = EventSelector_FrameStartOvertrigger;
            Camera.EventNotification.SetValue(EventNotification_Off);

            // Deregister the callback.
#if defined ( USE_USB )
            Camera.EventFrameStartOvertriggerTimestamp.GetNode()->DeregisterCallback(hCb2);
#else
            Camera.FrameStartOvertriggerEventTimestamp.GetNode()->DeregisterCallback(hCb2);
#endif
        }

        // Cleanup of event grabber and event adapter
        EventGrabber.Close();
        Camera.DestroyEventAdapter(pEventAdapter);


        // Cleanup of camera object
        Camera.Close();
    }
    catch (const GenericException &e)
    {
        // Error handling
        cerr << "An exception occurred!" << endl
        << e.GetDescription() << endl;
        pressEnterToExit();
        return 1;
    }
    // Quit the application.
    pressEnterToExit();
    return 0;
}

