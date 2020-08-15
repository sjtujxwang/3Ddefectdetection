// AcquireSingleFrame_Gen.cpp
//
/*
    Important: This sample shows how to migrate existing Low Level API code
    in order to be able to use USB3 Vision compliant cameras. Before migrating code,
    Basler strongly recommends reading the "Migrating from Previous Versions" topic in the pylon C++ API documentation
    that gets installed with pylon.

    The sample code below shows generic parameter access (any interface can be used at runtime).
    Furthermore the use of the Migration Mode Feature is shown.
*/

#include <pylon/PylonIncludes.h>

// Namespace for using cout
using namespace std;

// This function can be used to wait for user input at the end of the sample program.
void pressEnterToExit()
{
    //Comment the following two lines to disable wait on exit here.
    cerr << endl << "Press enter to exit." << endl;
    while( cin.get() != '\n');
}

// Namespace for using PYLON
using namespace Pylon;
using namespace GenApi;
using namespace GenICam;


int main(int argc, char* argv[])
{
    // Automagically call PylonInitialize and PylonTerminate to ensure that the pylon runtime
    // system is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Get the transport layer factory.
        CTlFactory& TlFactory = CTlFactory::GetInstance();

        // Get all attached cameras and exit the application if no camera is found.
        DeviceInfoList_t devices;
        if (0 == TlFactory.EnumerateDevices(devices))
        {
            cerr << "No camera present!" << endl;
            pressEnterToExit();
            return 1;
        }

        // Camera Link devices can't grab in pylon, so we can't use them in this sample.
        if (devices[0].GetDeviceClass() == Pylon::BaslerCameraLinkDeviceClass)
        {
            cerr << "This sample doesn't support " << Pylon::BaslerCameraLinkDeviceClass << " cameras!" << endl;
            pressEnterToExit();
            return 1;
        }

        // Create the camera object of the first available camera.
        // The camera object is used to set and get all available
        // camera features.
        IPylonDevice &Camera = *TlFactory.CreateDevice(devices[ 0 ]);

        // Open the camera.
        Camera.Open();

        // Check to see which Standard Feature Naming Convention (SFNC) is used by the camera device.
        if ( GetSfncVersion(Camera.GetNodeMap()) >= Sfnc_2_0_0)
        {
            // *****************************************************************
            // Enable Migration Mode, so that you can use many, but not all, old feature names on newer cameras.
            // To be compliant to the GigE Vision and USB 3 Vision standards, the naming of camera features has
            // to follow the naming conventions as defined by the GenICam �Standard Features Naming Convention (SFNC)�
            // document.
            // To be compliant with the USB 3 Vision standard, the feature names of a USB camera have to be chosen from
            // the version 2.0 of the SFNC document. Several feature names of the SFNC 2.0 differ from the names defined
            // by earlier versions of the SFNC.
            // To minimize the amount of code changes required to adapt existing code to the new naming schema, Basler
            // provides the Migration Mode, mapping old feature names to new ones.
            // Important: Basler strongly recommends reading the "Migrating from Previous Versions" topic in the pylon
            // C++ API documentation delivered with pylon. The topic contains a list of features covered by
            // the Migration Mode feature.
            INodeMap* pTLNodeMap = Camera.GetTLNodeMap(); // Access the Camera's Special "Transport Layer" nodemap
            if ( pTLNodeMap)
            {
                CBooleanPtr migrationModeEnable(pTLNodeMap->GetNode("MigrationModeEnable"));
                if ( GenApi::IsWritable( migrationModeEnable))
                {
                    // See usage of ExposureTimeRaw below for an example of old code that will not work unless Migration Mode is enabled.
                    migrationModeEnable->SetValue( true);
                }
            }
            // *****************************************************************
        }

        // Get the first stream grabber object of the selected camera.
        IStreamGrabber &StreamGrabber = *Camera.GetStreamGrabber(0);

        // Open the grabber.
        StreamGrabber.Open();

        // Get the stream parameter object.
        INodeMap &StreamParameters = *StreamGrabber.GetNodeMap();

        // Get the camera control object.
        INodeMap &Control = *Camera.GetNodeMap();

        // Get the AOI and payload size objects.
        const CIntegerPtr Width = Control.GetNode("Width");
        const CIntegerPtr Height = Control.GetNode("Height");
        CIntegerPtr PayloadSize = Control.GetNode("PayloadSize");
        if (!GenApi::IsAvailable(PayloadSize))
        {
            /* Note: Some camera devices, e.g Camera Link or BCON, don't have a payload size node.
                     In this case we'll look in the stream grabber node map for the PayloadSize node
                     The stream grabber, this can be a frame grabber, needs to be open to compute the
                     required payload size.
            */
            PayloadSize = StreamParameters.GetNode("PayloadSize");
        }

        // Set the image format and AOI.
        CEnumerationPtr pixelFormat(Control.GetNode("PixelFormat"));
        if (GenApi::IsAvailable(pixelFormat->GetEntryByName("Mono8")))
        {
            pixelFormat->FromString("Mono8");
        }
        CIntegerPtr(Control.GetNode("OffsetX"))->SetValue(0);
        CIntegerPtr(Control.GetNode("OffsetY"))->SetValue(0);
        Width->SetValue(Width->GetMax());
        Height->SetValue(Height->GetMax());

        //Disable all trigger types.
        {
            CEnumerationPtr triggerSelector(Control.GetNode("TriggerSelector"));

            //Disable acquisition start trigger if available.
            {
                GenApi::IEnumEntry* acquisitionStart = triggerSelector->GetEntryByName("AcquisitionStart");
                if ( acquisitionStart && GenApi::IsAvailable( acquisitionStart))
                {
                    triggerSelector->FromString("AcquisitionStart");
                    CEnumerationPtr(Control.GetNode("TriggerMode"))->FromString("Off");
                }
            }

            //Disable frame start trigger if available.
            {
                GenApi::IEnumEntry* frameStart = triggerSelector->GetEntryByName("FrameStart");
                if ( frameStart && GenApi::IsAvailable( frameStart))
                {
                    triggerSelector->FromString("FrameStart");
                    CEnumerationPtr(Control.GetNode("TriggerMode"))->FromString("Off");
                }
            }
        }

        //Set acquisition mode.
        CEnumerationPtr(Control.GetNode("AcquisitionMode"))->FromString("SingleFrame");

        //Set exposure settings.
        CEnumerationPtr(Control.GetNode("ExposureMode"))->FromString("Timed");

        // *****************************************************************
        // Migration Mode must be enabled if you wish to continue
        // to use ExposureTimeRaw. If Migration Mode is not enabled, you must now use
        // CFloatPtr(Control.GetNode("ExposureTime"))->SetValue(140);
        CIntegerPtr(Control.GetNode("ExposureTimeRaw"))->SetValue(140);
        // *****************************************************************

        // Create an image buffer.
        const size_t ImageSize = (size_t)(PayloadSize->GetValue());
        uint8_t * const pBuffer = new uint8_t[ ImageSize ];

        // We won't use image buffers greater than ImageSize.
        CIntegerPtr(StreamParameters.GetNode("MaxBufferSize"))->SetValue(ImageSize);

        // We won't queue more than one image buffer at a time.
        CIntegerPtr(StreamParameters.GetNode("MaxNumBuffer"))->SetValue(1);

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
            CCommandPtr(Control.GetNode("AcquisitionStart"))->Execute();

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

        // Delete camera object.
        TlFactory.DestroyDevice(&Camera);

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

    // Quit the application
    pressEnterToExit();
    return 0;
}

