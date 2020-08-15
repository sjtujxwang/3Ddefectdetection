#include "ros/ros.h"
#include "multi_robot_control/user_input.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "user_input");
    ros::NodeHandle nh;

    ros::ServiceClient user_input_client = nh.serviceClient<multi_robot_control::user_input>("user_input_service");
    multi_robot_control::user_input user_input_srv;
    user_input_srv.request.start = true;
    ROS_WARN("New car body comes!");
    ros::service::waitForService("user_input_service");
    if(user_input_client.call(user_input_srv))
    {
        ROS_INFO("the system is working!");
        if(user_input_srv.response.status == true)
        {
            ROS_INFO("The car body has been polished well!");
        }
    }
    return 0;
}