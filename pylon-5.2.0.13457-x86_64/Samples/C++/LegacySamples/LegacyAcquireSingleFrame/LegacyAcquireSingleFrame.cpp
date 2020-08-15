// AcquireSingleFrame.cpp
//
/*
    Important: This sample shows how to migrate existing Low Level API code
    in order to be able to use USB3 Vision compliant cameras. Before migrating code,
    Basler strongly recommends reading the "Migrating from Previous Versions" topic in the pylon C++ API documentation
    that gets installed with pylon.
*/

// Include files to use the pylon API.
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
#elif defined ( USE_USB )
// Settings to use Basler USB cameras
#include <pylon/usb/BaslerUsbCamera.h>
typedef Pylon::CBaslerUsbCamera Camera_t;
using namespace Basler_UsbCameraParams;
using namespace Basler_UsbStreamParams;
#elif defined ( USE_BCON )
// Settings to use Basler BCON cameras
#include <pylon/bcon/BaslerBconCamera.h>
typedef Pylon::CBaslerBconCamera Camera_t;
using namespace Basler_BconCameraParams;
using namespace Basler_BconStreamParams;
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

int main(int argc, char* argv[])
{
    // Automagically call PylonInitialize and PylonTerminate to ensure that the pylon runtime
    // system is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Get the transport layer factory.
        CTlFactory& TlFactory = CTlFactory::GetInstance();

        // Create the transport layer object needed to enumerate or
        // create a camera object of type Camera_t::DeviceClass().
        ITransportLayer *pTl = TlFactory.CreateTl(Camera_t::DeviceClass());

        // Exit the application if the specific transport layer is not available.
        if (! pTl)
        {
            cerr << "Failed to create transport layer!" << endl;
            pressEnterToExit();
            return 1;
        }

        // Get all attached cameras and exit the application if no camera is found.
        DeviceInfoList_t devices;
        if (0 == pTl->EnumerateDevices(devices))
        {
            cerr << "No camera present!" << endl;
            pressEnterToExit();
            return 1;
        }

        // Create the camera object of the first available camera.
        // The camera object is used to set and get all available
        // camera features.
        Camera_t Camera(pTl->CreateDevice(devices[0]));

        // Open the camera.
        Camera.Open();

        // Get the first stream grabber object of the selected camera.
        Camera_t::StreamGrabber_t StreamGrabber(Camera.GetStreamGrabber(0));

        // Open the stream grabber.
        StreamGrabber.Open();

        // Set the image format and AOI.
        if ( GenApi::IsAvailable( Camera.PixelFormat.GetEntry(PixelFormat_Mono8)))
        {
            Camera.PixelFormat.SetValue(PixelFormat_Mono8);
        }
        Camera.OffsetX.SetValue(0);
        Camera.OffsetY.SetValue(0);
        Camera.Width.SetValue(Camera.Width.GetMax());
        Camera.Height.SetValue(Camera.Height.GetMax());

#if !defined( USE_USB ) && !defined( USE_BCON )
        //Disable acquisition start trigger if available.
        {
            GenApi::IEnumEntry* acquisitionStart = Camera.TriggerSelector.GetEntry( TriggerSelector_AcquisitionStart);
            if ( acquisitionStart && GenApi::IsAvailable( acquisitionStart))
            {
                Camera.TriggerSelector.SetValue( TriggerSelector_AcquisitionStart);
                Camera.TriggerMode.SetValue( TriggerMode_Off);
            }
        }
#endif

        //Disable frame start trigger if available.
        {
            GenApi::IEnumEntry* frameStart = Camera.TriggerSelector.GetEntry( TriggerSelector_FrameStart);
            if ( frameStart && GenApi::IsAvailable( frameStart))
            {
                Camera.TriggerSelector.SetValue( TriggerSelector_FrameStart);
                Camera.TriggerMode.SetValue( TriggerMode_Off);
            }
        }

        //Set acquisition mode.
        Camera.AcquisitionMode.SetValue(AcquisitionMode_SingleFrame);

        // Set exposure settings.
        // (GigE and FireWire use ExposureTimeRaw due to previous Genicam SFNC version.)
        // (USB and BCON use ExposureTime due to Genicam SFNC v2.0.)
#if defined( USE_1394 ) || defined( USE_GIGE )
        Camera.ExposureMode.SetValue(ExposureMode_Timed);
        Camera.ExposureTimeRaw.SetValue(140);
#endif
#if defined( USE_USB ) || defined( USE_BCON )
        Camera.ExposureMode.SetValue(ExposureMode_Timed);
        Camera.ExposureTime.SetValue(140);
#endif

        // Create an image buffer.
#if defined( USE_BCON )
        const size_t ImageSize = (size_t)(StreamGrabber.PayloadSize.GetValue());
#else
        const size_t ImageSize = (size_t)(Camera.PayloadSize.GetValue());
#endif
        uint8_t * const pBuffer = new uint8_t[ ImageSize ];

        // We won't use image buffers greater than ImageSize.
        StreamGrabber.MaxBufferSize.SetValue(ImageSize);

        // We won't queue more than one image buffer at a time.
        StreamGrabber.MaxNumBuffer.SetValue(1);

        // Allocate all resources for grabbing. Critical parameters like image
        // size now must not be changed until FinishGrab() is called.
        StreamGrabber.PrepareGrab();

        // Buffers used for grabbing must be registered at the stream grabber.
        // The registration returns a handle to be used for queuing the buffer.
        const StreamBufferHandle hBuffer =
            StreamGrabber.RegisterBuffer(pBuffer, ImageSize);

        // Put the buffer into the grab queue for grabbing.
        StreamGrabber.QueueBuffer(hBuffer, NULL);

        // Grab 10 times.
        const uint32_t numGrabs = 10;
        for (uint32_t n = 0; n < numGrabs; n++)
        {
            // Let the camera acquire one single image ( Acquisition mode equals
            // SingleFrame! ).
            Camera.AcquisitionStart.Execute();

            // Wait for the grabbed image with a timeout of 3 seconds.
            if (StreamGrabber.GetWaitObject().Wait(3000))
            {
                // Get the grab result from the grabber's result queue.
                GrabResult Result;
                StreamGrabber.RetrieveResult(Result);

                if (Result.Succeeded())
                {
                    // Grabbing was successful, process image.
                    cout << "Image #" << n << " acquired!" << endl;
                    cout << "Size: " << Result.GetSizeX() << " x "
                    << Result.GetSizeY() << endl;

                    // Get the pointer to the image buffer.
                    const uint8_t *pImageBuffer = (uint8_t *) Result.Buffer();
                    cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0]
                    << endl << endl;

                    // Reuse the buffer for grabbing the next image.
                    if (n < numGrabs - 1)
                        StreamGrabber.QueueBuffer(Result.Handle(), NULL);
                }
                else
                {
                    // Error handling
                    cerr << "No image acquired!" << endl;
                    cerr << "Error code : 0x" << hex
                    << Result.GetErrorCode() << endl;
                    cerr << "Error description : "
                    << Result.GetErrorDescription() << endl;

                    // Cancel loop.
                    break;
                }
            }
            else
            {
                // Timeout
                cerr << "Timeout occurred!" << endl;

                // Get the pending buffer back (You are not allowed to deregister
                // buffers when they are still queued).
                StreamGrabber.CancelGrab();

                // Get all buffers back.
                for (GrabResult r; StreamGrabber.RetrieveResult(r););

                // Cancel loop.
                break;
            }
        }

        // Clean up

        // You must deregister the buffers before freeing the memory.
        StreamGrabber.DeregisterBuffer(hBuffer);

        // Free all resources used for grabbing.
        StreamGrabber.FinishGrab();

        // Close stream grabber.
        StreamGrabber.Close();

        // Close camera.
        Camera.Close();


        // Free memory of image buffer.
        delete[] pBuffer;
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
