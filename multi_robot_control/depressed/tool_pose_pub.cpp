// 发送两个机器人的末端位姿
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2_ros/transform_listener.h>
#include "multi_robot_control/pose.h"
#include "tf/transform_listener.h"
#include "iomanip"
#include "Eigen/Core"
#include "Eigen/Geometry"

void pose_convert(geometry_msgs::TransformStamped transformStamed, multi_robot_control::pose& pose)
{
    pose.pose[0] = transformStamed.transform.translation.x;
    pose.pose[1] = transformStamed.transform.translation.y;
    pose.pose[2] = transformStamed.transform.translation.z;

    Eigen::Quaterniond q;
    q.x() = transformStamed.transform.rotation.x;
    q.y() = transformStamed.transform.rotation.y;
    q.z() = transformStamed.transform.rotation.z;
    q.w() = transformStamed.transform.rotation.w;

    Eigen::AngleAxisd axis(q);
    double theta = axis.angle();

    pose.pose[3] = theta * axis.axis()[0];
    pose.pose[4] = theta * axis.axis()[1];
    pose.pose[5] = theta * axis.axis()[2];
}

void display(multi_robot_control::pose pose)
{
    using std::cout; 
    using std::endl;
    using std::setw;
    cout<<setw(10)<<pose.pose[0]<<setw(10)<<pose.pose[1]<<setw(10)<<pose.pose[2]
        <<setw(10)<<pose.pose[3]<<setw(10)<<pose.pose[4]<<setw(10)<<pose.pose[5]<<endl;
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tool_pose_pub");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped_ur1, transformStamped_ur2;
    multi_robot_control::pose ur1_pose, ur2_pose;
    ur1_pose.pose.resize(6);
    ur2_pose.pose.resize(6);

    ros::Publisher ur1_pose_pub = nh.advertise<multi_robot_control::pose>("ur1_tool_pose", 1000);
    ros::Publisher ur2_pose_pub = nh.advertise<multi_robot_control::pose>("ur2_tool_pose", 1000);
 
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        try
        {
            transformStamped_ur1 = tfBuffer.lookupTransform("base", "tool0_controller",ros::Time(0));
            transformStamped_ur2 = tfBuffer.lookupTransform("ur10_base", "ur10_tool0_controller",ros::Time(0));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        using std::cout;
        using std::endl;
        pose_convert(transformStamped_ur1, ur1_pose);
        pose_convert(transformStamped_ur2, ur2_pose);

        ur1_pose_pub.publish(ur1_pose);
        ur2_pose_pub.publish(ur2_pose);

        ROS_INFO("Tool0_controller pose of ur1 is");
        display(ur1_pose);
        ROS_INFO("Tool0_controller pose of ur2 is");
        display(ur2_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}