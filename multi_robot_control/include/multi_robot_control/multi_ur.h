#ifndef MULTI_UR_H
#define MULTI_UR_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "multi_robot_control/pose.h"
#include "iostream"


const int UR_JOINT = 6;
using std::string;
using std::cout;
using std::endl;


namespace multi_ur
{
    class ur1
    {
    public:
        ur1(ros::NodeHandle nh);
        bool Move(std::vector<double> pose_vector, double a = 0.5, double v = 0.5);

        std::vector<double> eef_pose;
    
    private:
        ros::Subscriber joint_sub;
        sensor_msgs::JointState joint_state_msgs;
        void JointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);

        ros::Subscriber tool0_pose_sub;
        multi_robot_control::pose tool0_pose_msgs;
        void Tool0PoseStateCallback(const multi_robot_control::poseConstPtr& tool0_pose);
        
        ros::Publisher pose_pub;
        std_msgs::String pose_msgs;
        
        std::string UrPoseCombine(std::vector<double> ur_pose, double a, double v);
        std::string double2string(double &input);
        double vector_distance(std::vector<double> pose_vector, multi_robot_control::pose pose);
    };

    class ur2
    {
    public:
        ur2(ros::NodeHandle nh);
        bool Move(std::vector<double> pose_vector, double a = 0.5, double v = 0.5); 

        std::vector<double> eef_pose;
    
    private:
        ros::Subscriber joint_sub;
        sensor_msgs::JointState joint_state_msgs;
        void JointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);

        ros::Subscriber tool0_pose_sub;
        multi_robot_control::pose tool0_pose_msgs;
        void Tool0PoseStateCallback(const multi_robot_control::poseConstPtr& tool0_pose);
        
        ros::Publisher pose_pub;
        std_msgs::String pose_msgs;
        
        std::string UrPoseCombine(std::vector<double> ur_pose, double a, double v);
        std::string double2string(double &input);
        double vector_distance(std::vector<double> pose_vector, multi_robot_control::pose pose);
        
    }; 

    class ur
    {
    public:
        ur(ros::NodeHandle nh);    
        ur1 ur_polish;
        ur2 ur_capture;
    };
}


#endif