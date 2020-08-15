// Grab_UsingExposureEndEvent_Usb.cpp
/*
    Note: Before getting started, Basler recommends reading the "Programmer's Guide" topic
    in the pylon C++ API documentation delivered with pylon.
    If you are upgrading to a higher major version of pylon, Basler also
    strongly recommends reading the "Migrating from Previous Versions" topic in the pylon C++ API documentation.

    This sample shows how to use the Exposure End event to speed up the image acquisition.
    For example, when a sensor exposure is finished, the camera can send an Exposure End event to the computer.
    The computer can receive the event before the image data of the finished exposure has been completely transferred.
    This can be used in order to avoid an unnecessary delay, for example when an imaged
    object is moved further before the related image data transfer is complete.
*/

// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>

// Include files used by samples.
#include "../include/ConfigurationEventPrinter.h"

#include <iomanip>

#ifdef PYLON_UNIX_BUILD
#    include <sys/time.h>
#endif

// Namespace for using pylon objects.
using namespace Pylon;

// Settings for using Basler USB cameras.
#include <pylon/usb/BaslerUsbInstantCamera.h>
typedef Pylon::CBaslerUsbInstantCamera Camera_t;
typedef Pylon::CBaslerUsbCameraEventHandler CameraEventHandler_t; // Or use Camera_t::CameraEventHandler_t
typedef Pylon::CBaslerUsbImageEventHandler ImageEventHandler_t; // Or use Camera_t::ImageEventHandler_t
typedef Pylon::CBaslerUsbGrabResultPtr GrabResultPtr_t; // Or use Camera_t::GrabResultPtr_t
using namespace Basler_UsbCameraParams;

// Namespace for using cout.
using namespace std;

// Enumeration used for distinguishing different events.
enum MyEvents
{
    eMyExposureEndEvent,      // Triggered by a camera event.
    eMyFrameStartOvertrigger, // Triggered by a camera event.
    eMyImageReceivedEvent,    // Triggered by the receipt of an image.
    eMyMoveEvent,             // Triggered when the imaged item or the sensor head can be moved.
    eMyNoEvent                // Used as default setting.
};

// Names of possible events for a printed output.
const char* MyEventNames[] =
{
    "ExposureEndEvent     ",
    "FrameStartOvertrigger",
    "ImageReceived        ",
    "Move                 ",
    "NoEvent              "
};

// Used for logging received events without outputting the information on the screen
// because outputting will change the timing.
// This class is used for demonstration purposes only.
struct LogItem
{
    LogItem()
        : eventType( eMyNoEvent)
        , frameNumber(0)
    {
    }

    LogItem( MyEvents event, uint16_t frameNr)
        : eventType(event)
        , frameNumber(frameNr)
    {
        //Warning, measured values can be wrong on older computer hardware.
#if defined(PYLON_WIN_BUILD)
        QueryPerformanceCounter(&time);
#elif defined(PYLON_UNIX_BUILD)
        struct timeval tv;

        gettimeofday(&tv, NULL);
        time = static_cast<unsigned long long>(tv.tv_sec) * 1000L + static_cast<unsigned long long>(tv.tv_usec) / 1000LL;
#endif
    }


#if defined(PYLON_WIN_BUILD)
    LARGE_INTEGER time; // Recorded time stamps.
#elif defined(PYLON_UNIX_BUILD)
    unsigned long long time; // Recorded time stamps.
#endif
    MyEvents eventType; // Type of the received event.
    uint16_t frameNumber; // Frame number of the received event.
};


// Helper function for printing a log.
// This function is used for demonstration purposes only.
void PrintLog( const std::vector<LogItem>& aLog)
{
#if defined(PYLON_WIN_BUILD)
    // Get the computer timer frequency.
    LARGE_INTEGER timerFrequency;
    QueryPerformanceFrequency(&timerFrequency);
#endif

    cout << std::endl << "Warning, the printed time values can be wrong on older computer hardware." << std::endl << std::endl;
    // Print the event information header.
    cout << "Time [ms]    " << "Event                 " << "FrameNumber" << std::endl;
    cout << "------------ " << "--------------------- " << "-----------" << std::endl;

    // Print the logged information.
    size_t logSize = aLog.size();
    for ( size_t i = 0; i < logSize; ++i)
    {
        // Calculate the elapsed time between the events.
        double time_ms = 0;
        if ( i)
        {
#if defined(PYLON_WIN_BUILD)
            __int64 oldTicks = ((__int64)aLog[i-1].time.HighPart << 32) + (__int64)aLog[i-1].time.LowPart;
            __int64 newTicks = ((__int64)aLog[i].time.HighPart << 32) + (__int64)aLog[i].time.LowPart;
            long double timeDifference = (long double) (newTicks - oldTicks);
            long double ticksPerSecond = (long double) (((__int64)timerFrequency.HighPart << 32) + (__int64)timerFrequency.LowPart);
            time_ms = (timeDifference / ticksPerSecond) * 1000;
#elif defined(PYLON_UNIX_BUILD)
            time_ms = aLog[i].time - aLog[i-1].time;
#endif
        }

        // Print the event information.
        cout << setw(12) << fixed << setprecision(4) << time_ms <<" "<< MyEventNames[ aLog[i].eventType ] <<" "<< aLog[i].frameNumber << std::endl;
    }
}


// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 50;


// Example handler for GigE camera events.
// Additional handling is required for GigE camera events because the event network packets can be lost, doubled or delayed on the network.
class CEventHandler : public CameraEventHandler_t, public ImageEventHandler_t
{
public:
    CEventHandler()
        : m_nextExpectedFrameNumberImage(0)
        , m_nextExpectedFrameNumberExposureEnd(0)
        , m_nextFrameNumberForMove(0)
        , m_frameIDsInitialized(false)
    {
        // Reserve space to log camera, image and move events.
        m_log.reserve( c_countOfImagesToGrab * 3);
    }

    // This method is called when a camera event has been received.
    virtual void OnCameraEvent( Camera_t& camera, intptr_t userProvidedId, GenApi::INode* /* pNode */)
    {
        if ( userProvidedId == eMyExposureEndEvent)
        {
            // An Exposure End event has been received.
            uint16_t frameNumber = (uint16_t)camera.EventExposureEndFrameID.GetValue();
            m_log.push_back( LogItem( eMyExposureEndEvent, frameNumber));

            // If Exposure End event is not doubled.
            if ( GetIncrementedFrameNumber( frameNumber) != m_nextExpectedFrameNumberExposureEnd)
            {
                // Check whether the imaged item or the sensor head can be moved.
                if ( frameNumber == m_nextFrameNumberForMove)
                {
                    MoveImagedItemOrSensorHead();
                }

                // Check for missing Exposure End events.
                if ( frameNumber != m_nextExpectedFrameNumberExposureEnd)
                {
                    throw RUNTIME_EXCEPTION( "An Exposure End event has been lost. Expected frame number is %d but got frame number %d.", m_nextExpectedFrameNumberExposureEnd, frameNumber);
                }
                IncrementFrameNumber( m_nextExpectedFrameNumberExposureEnd);
            }
        }
        else if ( userProvidedId == eMyFrameStartOvertrigger)
        {
            // The camera has been overtriggered.
            m_log.push_back( LogItem( eMyFrameStartOvertrigger, 0));

            // Handle this error...
        }
        else
        {
            PYLON_ASSERT2(false, "The sample has been modified and a new event has been registered. Add handler code above.");
        }
    }

    // This method is called when an image has been grabbed.
    virtual void OnImageGrabbed( Camera_t& camera, const GrabResultPtr_t& ptrGrabResult)
    {
        // An image has been received.
        uint16_t frameNumber = (uint16_t)ptrGrabResult->GetBlockID();
        m_log.push_back( LogItem( eMyImageReceivedEvent, frameNumber));

        // Check whether the imaged item or the sensor head can be moved.
        // This will be the case if the Exposure End has been lost or if the Exposure End is received later than the image.
        if ( frameNumber == m_nextFrameNumberForMove)
        {
            MoveImagedItemOrSensorHead();
        }

        // Check for missing images.
        if ( frameNumber != m_nextExpectedFrameNumberImage)
        {
            throw RUNTIME_EXCEPTION( "An image has been lost. Expected frame number is %d but got frame number %d.", m_nextExpectedFrameNumberImage, frameNumber);
        }
        IncrementFrameNumber( m_nextExpectedFrameNumberImage);
    }

    void MoveImagedItemOrSensorHead()
    {
        // The imaged item or the sensor head can be moved now...
        // The camera may not be ready for a trigger at this point yet because the sensor is still being read out.
        // See the documentation of the CInstantCamera::WaitForFrameTriggerReady() method for more information.
        m_log.push_back( LogItem( eMyMoveEvent, m_nextFrameNumberForMove));
        IncrementFrameNumber( m_nextFrameNumberForMove);
    }

    void PrintLog()
    {
        ::PrintLog( m_log);
    }

private:
    void IncrementFrameNumber( uint16_t& frameNumber)
    {
        frameNumber = GetIncrementedFrameNumber( frameNumber);
    }

    uint16_t GetIncrementedFrameNumber( uint16_t frameNumber)
    {
        ++frameNumber;
        return frameNumber;
    }

    uint16_t m_nextExpectedFrameNumberImage;
    uint16_t m_nextExpectedFrameNumberExposureEnd;
    uint16_t m_nextFrameNumberForMove;

    bool m_frameIDsInitialized;

    std::vector<LogItem> m_log;
};



int main(int argc, char* argv[])
{
    // Exit code of the sample application.
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
        // Create the event handler.
        CEventHandler eventHandler;

        // Only look for cameras supported by Camera_t.
        CDeviceInfo info;
        info.SetDeviceClass( Camera_t::DeviceClass());

        // Create an instant camera object with the first found camera device matching the specified device class.
        Camera_t camera( CTlFactory::GetInstance().CreateFirstDevice( info));

        // For demonstration purposes only, add sample configuration event handlers to print out information
        // about camera use and image grabbing.
        camera.RegisterConfiguration( new CConfigurationEventPrinter, RegistrationMode_Append, Cleanup_Delete); // Camera use.

        // Register the event handler.
        camera.RegisterImageEventHandler( &eventHandler, RegistrationMode_Append, Cleanup_None);
        camera.RegisterCameraEventHandler( &eventHandler, "EventExposureEndData", eMyExposureEndEvent, RegistrationMode_ReplaceAll, Cleanup_None);
        camera.RegisterCameraEventHandler( &eventHandler, "EventFrameStartOvertriggerData", eMyFrameStartOvertrigger, RegistrationMode_Append, Cleanup_None);

        // Camera event processing must be activated first, the default is off.
        camera.GrabCameraEvents = true;

        // Open the camera for setting parameters.
        camera.Open();

        // Check if the device supports events.
        if ( !IsAvailable( camera.EventSelector))
        {
            throw RUNTIME_EXCEPTION( "The device doesn't support events.");
        }

        // Enable the sending of Exposure End events.
        // Select the event to be received.
        camera.EventSelector.SetValue(EventSelector_ExposureEnd);
        // Enable it.
        camera.EventNotification.SetValue(EventNotification_On);

        // Enable the sending of Frame Start Overtrigger events.
        if ( IsAvailable( camera.EventSelector.GetEntry(EventSelector_FrameStartOvertrigger)))
        {
            camera.EventSelector.SetValue(EventSelector_FrameStartOvertrigger);
            camera.EventNotification.SetValue(EventNotification_On);
        }

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing( c_countOfImagesToGrab);

        // This smart pointer will receive the grab result data.
        CGrabResultPtr ptrGrabResult;

        // Camera.StopGrabbing() is called automatically by the RetrieveResult() method
        // when c_countOfImagesToGrab images have been retrieved.
        while ( camera.IsGrabbing())
        {
            // Retrieve grab results and notify the camera event and image event handlers.
            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
            // Nothing to do here with the grab result, the grab results are handled by the registered event handlers.
        }

        // Disable the sending of Exposure End events.
        camera.EventSelector = EventSelector_ExposureEnd;
        camera.EventNotification.SetValue(EventNotification_Off);

        // Disable the sending of Frame Start Overtrigger events.
        if ( IsAvailable( camera.EventSelector.GetEntry(EventSelector_FrameStartOvertrigger)))
        {
            camera.EventSelector.SetValue(EventSelector_FrameStartOvertrigger);
            camera.EventNotification.SetValue(EventNotification_Off);
        }

        // Print the recorded log showing the timing of events and images.
        eventHandler.PrintLog();
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
        << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press Enter to exit." << endl;
    while( cin.get() != '\n');

    // Releases all pylon resources. 
    PylonTerminate(); 

    return exitCode;
}

