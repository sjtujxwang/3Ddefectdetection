/*
    Important: This sample shows how to migrate existing Low Level API code
    in order to be able to use USB3 Vision compliant cameras. Before migrating code,
    Basler strongly recommends reading the "Migrating from Previous Versions" topic in the pylon C++ API documentation
    that gets installed with pylon.

    This sample program demonstrates how to be informed about the removal of a device.

    Attention:
    If you build this sample in debug mode and run it using a GigE camera device, pylon will set the heartbeat
    timeout to 5 minutes. This is done to allow debugging and single stepping of the code without
    the camera thinking we're hung because we don't send any heartbeats.
    This means that it would take 5 minutes for the application to notice the disconnection of a GigE device.

    To work around this, the CHeatbeatHelper class is used to control the HeartbeatTimeout.
    Just before waiting for the removal, it will set the timeout to 1000 ms.

*/

#include <pylon/PylonIncludes.h>
#include <iostream>

using namespace std;  // For cout

// This function can be used to wait for user input at the end of the sample program.
void pressEnterToExit()
{
    //Comment the following two lines to disable wait on exit here.
    cerr << endl << "Press enter to exit." << endl;
    while( cin.get() != '\n');
}

// Namespace for using pylon
using namespace Pylon;

static int callbackCounter(0);  // Will be incremented by the callback functions

#if defined (__GNUC__) && defined (__linux__)
#   include <time.h>
#   include <sys/time.h>
static void
Sleep(unsigned int ms)
{
    struct timeval now, end, delay;
    delay.tv_sec = ms / 1000;
    delay.tv_usec = (ms % 1000) * 1000;
    gettimeofday(&now, NULL);
    timeradd(&now, &delay, &end);
    while (timercmp(&now, &end, <))
    {
        struct timespec timeout, remain;
        timersub(&end, &now, &delay);
        timeout.tv_sec = delay.tv_sec;
        timeout.tv_nsec = delay.tv_usec * 1000;
        if (!nanosleep(&timeout, &remain))
            break;
        gettimeofday(&now, NULL);
    }
}
#endif

// Simple helper class to set the HeartbeatTimeout safely
class CHearbeatHelper
{
    public:
        explicit CHearbeatHelper(IPylonDevice* pCamera)
                : m_pHeartbeatTimeout(NULL)
        {
            try
            {
                if (pCamera == NULL)
                    return;

                GenApi::INodeMap* pTLNodeMap = pCamera->GetTLNodeMap();
                if (pTLNodeMap == NULL)
                    return; // Nothing we can do, leave m_pHeartbeatTimeout NULL

                // m_pHeartbeatTimeout may be NULL
                m_pHeartbeatTimeout = pTLNodeMap->GetNode("HeartbeatTimeout");
            }
            catch (const GenericException& e)
            {
                m_pHeartbeatTimeout = NULL;
                (void)e;
            }
        }

        bool SetValue(int64_t NewValue)
        {
            // Do nothing if no heartbeat feature is available.
            if (!m_pHeartbeatTimeout.IsValid())
                return false;

            // Apply the increment and cut off invalid values if necessary.
            int64_t correctedValue = NewValue - (NewValue % m_pHeartbeatTimeout->GetInc());

            m_pHeartbeatTimeout->SetValue(correctedValue);
            return true;
        }

        bool SetMax()
        {
            // Do nothing if no heartbeat feature is available.
            if (!m_pHeartbeatTimeout.IsValid())
                return false;

            int64_t maxVal = m_pHeartbeatTimeout->GetMax();
            return SetValue(maxVal);
        }

    protected:
        GenApi::CIntegerPtr m_pHeartbeatTimeout; // pointer to the node, will be NULL if no node exists
};


// A class with a member function that can be registered for device removal notification.
class AClass
{
    public:
        // The member function to be registered
        void RemovalCallbackMemberFunction(IPylonDevice* pDevice)
        {
            cout << endl << "Member function callback for removal of device "
            << pDevice->GetDeviceInfo().GetFullName().c_str() << " has been fired" << endl;
            callbackCounter++;
        }

};

void RemovalCallbackFunction(IPylonDevice* pDevice)
{
    cout << endl << "Callback function for removal of device "
    << pDevice->GetDeviceInfo().GetFullName().c_str() << " has been fired" << endl;
    callbackCounter++;
}

int main(int argc, char* argv[])
{
    // Automagically call PylonInitialize and PylonTerminate to ensure that the pylon runtime
    // system is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    AClass a;  // A member function of this class will be registered as a removal callback function.

    try
    {
        // Ask the user to disconnect a device.
        cout << "Looking for cameras ... ";

        // Get the transport layer factory
        CTlFactory& TlFactory = CTlFactory::GetInstance();

        // Get all attached cameras and exit the application if no camera is found.
        DeviceInfoList_t devices;
        if (0 == TlFactory.EnumerateDevices(devices))
        {
            cout << endl;
            cerr << "No camera present!" << endl;
            pressEnterToExit();
            return 1;
        }
        else
        {
            cout << "done." << endl;
        }

        // Create the camera object of the first available camera.
        // The camera object is used to set and get all available
        // camera features.
        cout << "Opening camera " << devices[0].GetFriendlyName() << " (" << devices[0].GetFullName() << ") ..." << endl;
        IPylonDevice* pCamera = TlFactory.CreateDevice(devices[ 0 ]);

        // Open the camera
        pCamera->Open();

        // Register a member function
        DeviceCallbackHandle hCb1 = RegisterRemovalCallback(pCamera, a, &AClass::RemovalCallbackMemberFunction);

        // Register a "normal" function
        DeviceCallbackHandle hCb2 = RegisterRemovalCallback(pCamera, &RemovalCallbackFunction);

        // Ask the user to disconnect a device
        int loopCount = 20 * 4;
        cout << "Please disconnect the device (timeout " << loopCount / 4 << "s) ";
        fflush(stdout);

        /////////////////////////////////////////////////// don't single step behind this line  (see comments above)
        // Before testing the callbacks, for GigE cameras we manually set the heartbeat timeout to a short value.
        // Since for debug versions the heartbeat timeout has been set to 5 minutes, it would take up to 5 minutes
        // until detection of the device removal.
        CHearbeatHelper heartbeatHelper(pCamera);
        heartbeatHelper.SetValue(1000);  // 1000 ms timeout

        // Wait until the removal has been registered and both registered callback functions have been fired.
        do
        {
            // Print a . every few seconds to tell the user we're waiting for the callback
            if (--loopCount % 4 == 0)
            {
                cout << ".";
                fflush(stdout);
            }
            Sleep(250);
        }
        while (callbackCounter < 1 && loopCount >= 0);  /* Check loopCount so we won't wait forever */

        if (callbackCounter < 2)
            cout << endl << "Timeout expired" << endl;

        // Deregister the callback functions (must not be done after closing the camera object)
        if (! pCamera->DeregisterRemovalCallback(hCb1))
            cerr << "Failed to deregister the callback function" << endl;
        if (! pCamera->DeregisterRemovalCallback(hCb2))
            cerr << "Failed to deregister the callback function" << endl;

        // Close the camera object
        pCamera->Close();

        // Destroy the camera
        TlFactory.DestroyDevice(pCamera);
        pCamera = NULL;

    }
    catch (const GenericException &e)
    {
        // Error handling
        cerr << "An exception occurred!" << endl
        << e.GetDescription() << endl;
        pressEnterToExit();
        return 1;
    }

    pressEnterToExit();
    return 0;
}
