// ParametrizeCamera_AutoFunctions.cpp
/*
    Note: Before getting started, Basler recommends reading the "Programmer's Guide" topic
    in the pylon C++ API documentation delivered with pylon.
    If you are upgrading to a higher major version of pylon, Basler also
    strongly recommends reading the "Migrating from Previous Versions" topic in the pylon C++ API documentation.

    This sample illustrates how to use the Auto Functions feature of Basler cameras.
*/

// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

#if defined( USE_1394 )
// Settings to use  Basler IEEE 1394 cameras.
#include <pylon/1394/Basler1394InstantCamera.h>
typedef Pylon::CBasler1394InstantCamera Camera_t;
using namespace Basler_IIDC1394CameraParams;
#elif defined ( USE_GIGE )
// Settings to use Basler GigE cameras.
#include <pylon/gige/BaslerGigEInstantCamera.h>
typedef Pylon::CBaslerGigEInstantCamera Camera_t;
using namespace Basler_GigECameraParams;
#else
#error camera type is not specified. For example, define USE_GIGE for using GigE cameras
#endif

// The camera specific grab result smart pointer.
typedef Camera_t::GrabResultPtr_t GrabResultPtr_t;

bool IsColorCamera(Camera_t& camera);
void AutoGainOnce(Camera_t& camera);
void AutoGainContinuous(Camera_t& camera);
void AutoExposureOnce(Camera_t& camera);
void AutoExposureContinuous(Camera_t& camera);
void AutoWhiteBalance(Camera_t& camera);

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
        // Only look for cameras supported by Camera_t.
        CDeviceInfo info;
        info.SetDeviceClass( Camera_t::DeviceClass());

        // Create an instant camera object with the first found camera device that matches the specified device class.
        Camera_t camera( CTlFactory::GetInstance().CreateFirstDevice( info));

        // Print the name of the used camera.
        cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

        // Register the standard event handler for configuring single frame acquisition.
        // This overrides the default configuration as all event handlers are removed by setting the registration mode to RegistrationMode_ReplaceAll.
        // Please note that the camera device auto functions do not require grabbing by single frame acquisition.
        // All available acquisition modes can be used.
        camera.RegisterConfiguration( new CAcquireSingleFrameConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);

        // Open the camera.
        camera.Open();

        // Turn test image off.
        camera.TestImageSelector = TestImageSelector_Off;

        // Only area scan cameras support auto functions.
        if (camera.DeviceScanType.GetValue() == DeviceScanType_Areascan)
        {
            // All area scan cameras support luminance control.

            // Carry out luminance control by using the "once" gain auto function.
            // For demonstration purposes only, set the gain to an initial value.
            camera.GainRaw.SetValue( camera.GainRaw.GetMax());
            AutoGainOnce(camera);
            cerr << endl << "Press Enter to continue." << endl;
            while( cin.get() != '\n');


            // Carry out luminance control by using the "continuous" gain auto function.
            // For demonstration purposes only, set the gain to an initial value.
            camera.GainRaw.SetValue( camera.GainRaw.GetMax());
            AutoGainContinuous(camera);
            cerr << endl << "Press Enter to continue." << endl;
            while( cin.get() != '\n');


            // For demonstration purposes only, set the exposure time to an initial value.
            camera.ExposureTimeRaw.SetValue( camera.ExposureTimeRaw.GetMin());

            // Carry out luminance control by using the "once" exposure auto function.
            AutoExposureOnce(camera);
            cerr << endl << "Press Enter to continue." << endl;
            while( cin.get() != '\n');


            // For demonstration purposes only, set the exposure time to an initial value.
            camera.ExposureTimeRaw.SetValue( camera.ExposureTimeRaw.GetMin());

            // Carry out luminance control by using the "continuous" exposure auto function.
            AutoExposureContinuous(camera);

            // Only color cameras support the balance white auto function.
            if (IsColorCamera(camera))
            {
                cerr << endl << "Press Enter to continue." << endl;
                while( cin.get() != '\n');

                // For demonstration purposes only, set the initial balance ratio values:
                camera.BalanceRatioSelector.SetValue(BalanceRatioSelector_Red);
                camera.BalanceRatioAbs.SetValue(3.14);
                camera.BalanceRatioSelector.SetValue(BalanceRatioSelector_Green);
                camera.BalanceRatioAbs.SetValue(0.5);
                camera.BalanceRatioSelector.SetValue(BalanceRatioSelector_Blue);
                camera.BalanceRatioAbs.SetValue(0.125);

                // Carry out white balance using the balance white auto function.
                AutoWhiteBalance(camera);
            }
        }
        else
        {
            cerr << "Only area scan cameras support auto functions." << endl;
        }

        // Close camera.
        camera.Close();

    }
    catch (const TimeoutException &e)
    {
        // Auto functions did not finish in time.
        // Maybe the cap on the lens is still on or there is not enough light.
        cerr << "A timeout has occurred." << endl
             << e.GetDescription() << endl;
        cerr << "Please make sure you remove the cap from the camera lens before running this sample." << endl;
        exitCode = 0;
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


void AutoGainOnce(Camera_t& camera)
{
    // Check whether the gain auto function is available.
    if ( !IsWritable( camera.GainAuto))
    {
        cout << "The camera does not support Gain Auto." << endl << endl;
        return;
    }

    // Maximize the grabbed image area of interest (Image AOI).
    if (IsWritable(camera.OffsetX))
    {
        camera.OffsetX.SetValue(camera.OffsetX.GetMin());
    }
    if (IsWritable(camera.OffsetY))
    {
        camera.OffsetY.SetValue(camera.OffsetY.GetMin());
    }
    camera.Width.SetValue(camera.Width.GetMax());
    camera.Height.SetValue(camera.Height.GetMax());

    // Set the Auto Function AOI for luminance statistics.
    // Currently, AutoFunctionAOISelector_AOI1 is predefined to gather
    // luminance statistics.
    camera.AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI1);
    camera.AutoFunctionAOIOffsetX.SetValue(camera.OffsetX.GetMin());
    camera.AutoFunctionAOIOffsetY.SetValue(camera.OffsetY.GetMin());
    camera.AutoFunctionAOIWidth.SetValue(camera.Width.GetMax());
    camera.AutoFunctionAOIHeight.SetValue(camera.Height.GetMax());

    // Set the target value for luminance control. The value is always expressed
    // as an 8 bit value regardless of the current pixel data output format,
    // i.e., 0 -> black, 255 -> white.
    camera.AutoTargetValue.SetValue(80);

    // We are going to try GainAuto = Once.

    cout << "Trying 'GainAuto = Once'." << endl;
    cout << "Initial Gain = " << camera.GainRaw.GetValue() << endl;

    // Set the gain ranges for luminance control.
    camera.AutoGainRawLowerLimit.SetValue(camera.GainRaw.GetMin());
    camera.AutoGainRawUpperLimit.SetValue(camera.GainRaw.GetMax());

    camera.GainAuto.SetValue(GainAuto_Once);

    // When the "once" mode of operation is selected,
    // the parameter values are automatically adjusted until the related image property
    // reaches the target value. After the automatic parameter value adjustment is complete, the auto
    // function will automatically be set to "off" and the new parameter value will be applied to the
    // subsequently grabbed images.

    int n = 0;
    while (camera.GainAuto.GetValue() != GainAuto_Off)
    {
        GrabResultPtr_t ptrGrabResult;
        camera.GrabOne( 5000, ptrGrabResult);
#ifdef PYLON_WIN_BUILD
        Pylon::DisplayImage(1, ptrGrabResult);
#endif
        ++n;
        //For demonstration purposes only. Wait until the image is shown.
        WaitObject::Sleep(100);

        //Make sure the loop is exited.
        if (n > 100)
        {
            throw TIMEOUT_EXCEPTION( "The adjustment of auto gain did not finish.");
        }
    }

    cout << "GainAuto went back to 'Off' after " << n << " frames." << endl;
    cout << "Final Gain = " << camera.GainRaw.GetValue() << endl << endl;
}


void AutoGainContinuous(Camera_t& camera)
{
    // Check whether the Gain Auto feature is available.
    if ( !IsWritable( camera.GainAuto))
    {
        cout << "The camera does not support Gain Auto." << endl << endl;
        return;
    }

    // Maximize the grabbed image area of interest (Image AOI).
    if (IsWritable(camera.OffsetX))
    {
        camera.OffsetX.SetValue(camera.OffsetX.GetMin());
    }
    if (IsWritable(camera.OffsetY))
    {
        camera.OffsetY.SetValue(camera.OffsetY.GetMin());
    }
    camera.Width.SetValue(camera.Width.GetMax());
    camera.Height.SetValue(camera.Height.GetMax());

    // Set the Auto Function AOI for luminance statistics.
    // Currently, AutoFunctionAOISelector_AOI1 is predefined to gather
    // luminance statistics.
    camera.AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI1);
    camera.AutoFunctionAOIOffsetX.SetValue(camera.OffsetX.GetMin());
    camera.AutoFunctionAOIOffsetY.SetValue(camera.OffsetY.GetMin());
    camera.AutoFunctionAOIWidth.SetValue(camera.Width.GetMax());
    camera.AutoFunctionAOIHeight.SetValue(camera.Height.GetMax());

    // Set the target value for luminance control. The value is always expressed
    // as an 8 bit value regardless of the current pixel data output format,
    // i.e., 0 -> black, 255 -> white.
    camera.AutoTargetValue.SetValue(80);

    // We are trying GainAuto = Continuous.
    cout << "Trying 'GainAuto = Continuous'." << endl;
    cout << "Initial Gain = " << camera.GainRaw.GetValue() << endl;

    camera.GainAuto.SetValue(GainAuto_Continuous);

    // When "continuous" mode is selected, the parameter value is adjusted repeatedly while images are acquired.
    // Depending on the current frame rate, the automatic adjustments will usually be carried out for
    // every or every other image unless the camera�s micro controller is kept busy by other tasks.
    // The repeated automatic adjustment will proceed until the "once" mode of operation is used or
    // until the auto function is set to "off", in which case the parameter value resulting from the latest
    // automatic adjustment will operate unless the value is manually adjusted.
    for (int n = 0; n < 20; n++)            // For demonstration purposes, we will grab "only" 20 images.
    {
        GrabResultPtr_t ptrGrabResult;
        camera.GrabOne( 5000, ptrGrabResult);
#ifdef PYLON_WIN_BUILD
        Pylon::DisplayImage(1, ptrGrabResult);
#endif

        //For demonstration purposes only. Wait until the image is shown.
        WaitObject::Sleep(100);
    }
    camera.GainAuto.SetValue(GainAuto_Off); // Switch off GainAuto.

    cout << "Final Gain = " << camera.GainRaw.GetValue() << endl << endl;
}


void AutoExposureOnce(Camera_t& camera)
{
    // Check whether auto exposure is available
    if ( !IsWritable( camera.ExposureAuto))
    {
        cout << "The camera does not support Exposure Auto." << endl << endl;
        return;
    }

    // Maximize the grabbed area of interest (Image AOI).
    if (IsWritable(camera.OffsetX))
    {
        camera.OffsetX.SetValue(camera.OffsetX.GetMin());
    }
    if (IsWritable(camera.OffsetY))
    {
        camera.OffsetY.SetValue(camera.OffsetY.GetMin());
    }
    camera.Width.SetValue(camera.Width.GetMax());
    camera.Height.SetValue(camera.Height.GetMax());

    // Set the Auto Function AOI for luminance statistics.
    // Currently, AutoFunctionAOISelector_AOI1 is predefined to gather
    // luminance statistics.
    camera.AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI1);
    camera.AutoFunctionAOIOffsetX.SetValue(camera.OffsetX.GetMin());
    camera.AutoFunctionAOIOffsetY.SetValue(camera.OffsetY.GetMin());
    camera.AutoFunctionAOIWidth.SetValue(camera.Width.GetMax());
    camera.AutoFunctionAOIHeight.SetValue(camera.Height.GetMax());

    // Set the target value for luminance control. The value is always expressed
    // as an 8 bit value regardless of the current pixel data output format,
    // i.e., 0 -> black, 255 -> white.
    camera.AutoTargetValue.SetValue(80);

    // Try ExposureAuto = Once.
    cout << "Trying 'ExposureAuto = Once'." << endl;
    cout << "Initial exposure time = ";
    cout << camera.ExposureTimeAbs.GetValue() << " us" << endl;

    // Set the exposure time ranges for luminance control.
    camera.AutoExposureTimeAbsLowerLimit.SetValue(camera.AutoExposureTimeAbsLowerLimit.GetMin());
    camera.AutoExposureTimeAbsUpperLimit.SetValue(camera.AutoExposureTimeAbsLowerLimit.GetMax());

    camera.ExposureAuto.SetValue(ExposureAuto_Once);

    // When the "once" mode of operation is selected,
    // the parameter values are automatically adjusted until the related image property
    // reaches the target value. After the automatic parameter value adjustment is complete, the auto
    // function will automatically be set to "off", and the new parameter value will be applied to the
    // subsequently grabbed images.
    int n = 0;
    while (camera.ExposureAuto.GetValue() != ExposureAuto_Off)
    {
        GrabResultPtr_t ptrGrabResult;
        camera.GrabOne( 5000, ptrGrabResult);
#ifdef PYLON_WIN_BUILD
        Pylon::DisplayImage(1, ptrGrabResult);
#endif
        ++n;

        //For demonstration purposes only. Wait until the image is shown.
        WaitObject::Sleep(100);

        //Make sure the loop is exited.
        if (n > 100)
        {
            throw TIMEOUT_EXCEPTION( "The adjustment of auto exposure did not finish.");
        }
    }
    cout << "ExposureAuto went back to 'Off' after " << n << " frames." << endl;
    cout << "Final exposure time = ";
    cout << camera.ExposureTimeAbs.GetValue() << " us" << endl << endl;
}


void AutoExposureContinuous(Camera_t& camera)
{
    // Check whether the Exposure Auto feature is available.
    if ( !IsWritable( camera.ExposureAuto))
    {
        cout << "The camera does not support Exposure Auto." << endl << endl;
        return;
    }

    // Maximize the grabbed area of interest (Image AOI).
    if (IsWritable(camera.OffsetX))
    {
        camera.OffsetX.SetValue(camera.OffsetX.GetMin());
    }
    if (IsWritable(camera.OffsetY))
    {
        camera.OffsetY.SetValue(camera.OffsetY.GetMin());
    }
    camera.Width.SetValue(camera.Width.GetMax());
    camera.Height.SetValue(camera.Height.GetMax());

    // Set the Auto Function AOI for luminance statistics.
    // Currently, AutoFunctionAOISelector_AOI1 is predefined to gather
    // luminance statistics.
    camera.AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI1);
    camera.AutoFunctionAOIOffsetX.SetValue(camera.OffsetX.GetMin());
    camera.AutoFunctionAOIOffsetY.SetValue(camera.OffsetY.GetMin());
    camera.AutoFunctionAOIWidth.SetValue(camera.Width.GetMax());
    camera.AutoFunctionAOIHeight.SetValue(camera.Height.GetMax());

    // Set the target value for luminance control. The value is always expressed
    // as an 8 bit value regardless of the current pixel data output format,
    // i.e., 0 -> black, 255 -> white.
    camera.AutoTargetValue.SetValue(80);

    cout << "ExposureAuto 'GainAuto = Continuous'." << endl;
    cout << "Initial exposure time = ";
    cout << camera.ExposureTimeAbs.GetValue() << " us" << endl;

    camera.ExposureAuto.SetValue(ExposureAuto_Continuous);

    // When "continuous" mode is selected, the parameter value is adjusted repeatedly while images are acquired.
    // Depending on the current frame rate, the automatic adjustments will usually be carried out for
    // every or every other image, unless the camera�s microcontroller is kept busy by other tasks.
    // The repeated automatic adjustment will proceed until the "once" mode of operation is used or
    // until the auto function is set to "off", in which case the parameter value resulting from the latest
    // automatic adjustment will operate unless the value is manually adjusted.
    for (int n = 0; n < 20; n++)    // For demonstration purposes, we will use only 20 images.
    {
        GrabResultPtr_t ptrGrabResult;
        camera.GrabOne( 5000, ptrGrabResult);
#ifdef PYLON_WIN_BUILD
        Pylon::DisplayImage(1, ptrGrabResult);
#endif

        //For demonstration purposes only. Wait until the image is shown.
        WaitObject::Sleep(100);
    }
    camera.ExposureAuto.SetValue(ExposureAuto_Off); // Switch off Exposure Auto.

    cout << "Final exposure time = ";
    cout << camera.ExposureTimeAbs.GetValue() << " us" << endl << endl;
}


void AutoWhiteBalance(Camera_t& camera)
{
    // Check whether the Balance White Auto feature is available.
    if ( !IsWritable( camera.BalanceWhiteAuto))
    {
        cout << "The camera does not support Balance White Auto." << endl << endl;
        return;
    }

    // Maximize the grabbed area of interest (Image AOI).
    if (IsWritable(camera.OffsetX))
    {
        camera.OffsetX.SetValue(camera.OffsetX.GetMin());
    }
    if (IsWritable(camera.OffsetY))
    {
        camera.OffsetY.SetValue(camera.OffsetY.GetMin());
    }
    camera.Width.SetValue(camera.Width.GetMax());
    camera.Height.SetValue(camera.Height.GetMax());

    // Set the Auto Function AOI for white balance statistics.
    // Currently, AutoFunctionAOISelector_AOI2 is predefined to gather
    // white balance statistics.
    camera.AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI2);
    camera.AutoFunctionAOIOffsetX.SetValue(camera.OffsetX.GetMin());
    camera.AutoFunctionAOIOffsetY.SetValue(camera.OffsetY.GetMin());
    camera.AutoFunctionAOIWidth.SetValue(camera.Width.GetMax());
    camera.AutoFunctionAOIHeight.SetValue(camera.Height.GetMax());

    cout << "Trying 'BalanceWhiteAuto = Once'." << endl;
    cout << "Initial balance ratio: ";
    camera.BalanceRatioSelector.SetValue(BalanceRatioSelector_Red);
        cout << "R = " << camera.BalanceRatioAbs.GetValue() << "   ";
    camera.BalanceRatioSelector.SetValue(BalanceRatioSelector_Green);
        cout << "G = " << camera.BalanceRatioAbs.GetValue() << "   ";
    camera.BalanceRatioSelector.SetValue(BalanceRatioSelector_Blue);
        cout << "B = " << camera.BalanceRatioAbs.GetValue() << endl;

    camera.BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Once);

    // When the "once" mode of operation is selected,
    // the parameter values are automatically adjusted until the related image property
    // reaches the target value. After the automatic parameter value adjustment is complete, the auto
    // function will automatically be set to "off" and the new parameter value will be applied to the
    // subsequently grabbed images.
    int n = 0;
    while (camera.BalanceWhiteAuto.GetValue() != BalanceWhiteAuto_Off)
    {
        GrabResultPtr_t ptrGrabResult;
        camera.GrabOne( 5000, ptrGrabResult);
#ifdef PYLON_WIN_BUILD
        Pylon::DisplayImage(1, ptrGrabResult);
#endif
        ++n;

        //For demonstration purposes only. Wait until the image is shown.
        WaitObject::Sleep(100);

        //Make sure the loop is exited.
        if (n > 100)
        {
            throw TIMEOUT_EXCEPTION( "The adjustment of auto white balance did not finish.");
        }
    }
    cout << "BalanceWhiteAuto went back to 'Off' after ";
    cout << n << " frames." << endl;
    cout << "Final balance ratio: ";
    camera.BalanceRatioSelector.SetValue(BalanceRatioSelector_Red);
    cout << "R = " << camera.BalanceRatioAbs.GetValue() << "   ";
    camera.BalanceRatioSelector.SetValue(BalanceRatioSelector_Green);
    cout << "G = " << camera.BalanceRatioAbs.GetValue() << "   ";
    camera.BalanceRatioSelector.SetValue(BalanceRatioSelector_Blue);
    cout << "B = " << camera.BalanceRatioAbs.GetValue() << endl;
}


bool IsColorCamera(Camera_t& camera)
{
    GenApi::NodeList_t Entries;
    camera.PixelFormat.GetEntries(Entries);
    bool Result = false;

    for (size_t i = 0; i < Entries.size(); i++)
    {
        GenApi::INode *pNode = Entries[i];
        if (IsAvailable(pNode->GetAccessMode()))
        {
            GenApi::IEnumEntry *pEnum = dynamic_cast<GenApi::IEnumEntry *>(pNode);
            const GenICam::gcstring sym(pEnum->GetSymbolic());
            if (sym.find(GenICam::gcstring("Bayer")) != GenICam::gcstring::_npos())
            {
                Result = true;
                break;
            }
        }
    }
    return Result;
}
