// 发送两个机器人的末端位姿
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2_ros/transform_listener.h>
#include "tf/transform_listener.h"
#include "iomanip"

void pose_convert(geometry_msgs::TransformStamped transformStamed, geometry_msgs::Pose& pose)
{
    pose.position.x = transformStamed.transform.translation.x;
    pose.position.y = transformStamed.transform.translation.y;
    pose.position.z = transformStamed.transform.translation.z;
    pose.orientation = transformStamed.transform.rotation;
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tool_pose_pub");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped_polish_robot, transformStamped_capture_robot;
    geometry_msgs::Pose capture_robot_eef_pose, polish_robot_eef_pose;

    ros::Publisher polish_robot_pose_pub = nh.advertise<geometry_msgs::Pose>("polish_robot_tool_pose", 1000);
    ros::Publisher capture_robot_pose_pub = nh.advertise<geometry_msgs::Pose>("capture_robot_tool_pose", 1000);
 
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        try
        {
            transformStamped_polish_robot = tfBuffer.lookupTransform("base", "tool0_controller",ros::Time(0));
            transformStamped_capture_robot = tfBuffer.lookupTransform("ur10_base", "ur10_tool0_controller",ros::Time(0));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        pose_convert(transformStamped_polish_robot, polish_robot_eef_pose);
        pose_convert(transformStamped_capture_robot, capture_robot_eef_pose);

        polish_robot_pose_pub.publish(polish_robot_eef_pose);
        capture_robot_pose_pub.publish(capture_robot_eef_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}