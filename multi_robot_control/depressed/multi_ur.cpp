// 和段师兄通讯时，实际机器人的控制程序

#include "multi_robot_control/multi_ur.h"
namespace multi_ur
{
    ur::ur(ros::NodeHandle nh):ur_polish(nh), ur_capture(nh)
    {
        ROS_INFO("Initializing for two ur robot!");
    }

    ur1::ur1(ros::NodeHandle nh)
    {
        joint_sub = nh.subscribe("/joint_states", 1000, &ur1::JointStateCallback, this);
        tool0_pose_sub = nh.subscribe("ur1_tool_pose", 1000, &ur1::Tool0PoseStateCallback, this);
        pose_pub = nh.advertise<std_msgs::String>("/ur1/ur_driver/URScript", 1000);
        joint_state_msgs.name.resize(UR_JOINT);
        joint_state_msgs.position.resize(UR_JOINT);
        joint_state_msgs.velocity.resize(UR_JOINT);
        tool0_pose_msgs.pose.resize(6);
        eef_pose.resize(6);

        ros::Duration(1.0).sleep();
    }

    bool ur1::Move(std::vector<double> pose_vector, double a, double v)
    {
        pose_msgs.data = UrPoseCombine(pose_vector, a, v);
        pose_pub.publish(pose_msgs);

        ros::Rate loop_rate(30);
        while (ros::ok())
        {
            if(vector_distance(pose_vector, tool0_pose_msgs) < 0.001)
            {
                break;
            }   
            ros::spinOnce();
            loop_rate.sleep();
        }
        return true;
    }
    void ur1::JointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
    {
        if(joint_state->name[0] == "shoulder_pan_joint")
        {
            // ROS_INFO("Hello1");
            joint_state_msgs.header.stamp = ros::Time::now();
            for(int i = 0; i < UR_JOINT; i ++)
            {
                joint_state_msgs.name[i] = joint_state->name[i];
                joint_state_msgs.position[i] = joint_state->position[i];
            }
        }
    }

    void ur1::Tool0PoseStateCallback(const multi_robot_control::poseConstPtr& tool0_pose)
    {
        tool0_pose_msgs = * tool0_pose;
        // ROS_INFO("Receving tool0 pose of ur1!");
    }

    double ur1::vector_distance(std::vector<double> pose_vector, multi_robot_control::pose pose)
    {
        double distance = 0;
        for(int i = 0; i < 3; i ++)
        {
            distance += (pose_vector[i] - pose.pose[i]) * (pose_vector[i] - pose.pose[i]);
        }
        return distance;
    }

    std::string ur1::UrPoseCombine(std::vector<double> ur_pose, double a, double v)
    {   
        using namespace std;
        std::string move_msg;
        move_msg = "movel(p[";
        move_msg = move_msg + double2string(ur_pose[0]) + ",";
        move_msg = move_msg + double2string(ur_pose[1]) + ",";
        move_msg = move_msg + double2string(ur_pose[2]) + ",";
        move_msg = move_msg + double2string(ur_pose[3]) + ",";
        move_msg = move_msg + double2string(ur_pose[4]) + ",";
        move_msg = move_msg + double2string(ur_pose[5]) + "]";
        move_msg = move_msg + ",a=";
        move_msg = move_msg + double2string(a) + ",v=";
        move_msg = move_msg + double2string(v) + ")";
        move_msg = move_msg + "\n";
        return move_msg; 
    }

    std::string ur1::double2string(double &input)
    {
        std::string string_temp;
        std::stringstream stream;
        stream<<input;
        string_temp = stream.str();
        return string_temp;
    }

    ur2::ur2(ros::NodeHandle nh)
    {
        joint_sub = nh.subscribe("/joint_states", 1000, &ur2::JointStateCallback, this);
        tool0_pose_sub = nh.subscribe("ur2_tool_pose", 1000, &ur2::Tool0PoseStateCallback, this);
        pose_pub = nh.advertise<std_msgs::String>("/ur2/ur_driver/URScript", 1000);
        joint_state_msgs.name.resize(UR_JOINT);
        joint_state_msgs.position.resize(UR_JOINT);
        joint_state_msgs.velocity.resize(UR_JOINT);
        tool0_pose_msgs.pose.resize(6);
        eef_pose.resize(6);

        ros::Duration(1.0).sleep();
        
    }

    bool ur2::Move(std::vector<double> pose_vector, double a, double v)
    {
        pose_msgs.data = UrPoseCombine(pose_vector, a, v);
        pose_pub.publish(pose_msgs);

        ros::Rate loop_rate(30);
        while (ros::ok())
        {
            if(vector_distance(pose_vector, tool0_pose_msgs) < 0.001)
            {
                break;
            }   
            ros::spinOnce();
            loop_rate.sleep();
        }
        return true;
    }

    void ur2::JointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
    {
        if(joint_state->name[0] == "ur10_shoulder_pan_joint")
        {
            // ROS_INFO("Hello1");
            joint_state_msgs.header.stamp = ros::Time::now();
            for(int i = 0; i < UR_JOINT; i ++)
            {
                joint_state_msgs.name[i] = joint_state->name[i];
                joint_state_msgs.position[i] = joint_state->position[i];
            }
        }
    }

    void ur2::Tool0PoseStateCallback(const multi_robot_control::poseConstPtr& tool0_pose)
    {
        tool0_pose_msgs = * tool0_pose;
        // ROS_INFO("Receving tool0 pose of ur2!");
    }

    double ur2::vector_distance(std::vector<double> pose_vector, multi_robot_control::pose pose)
    {
        double distance = 0;
        for(int i = 0; i < 3; i ++)
        {
            distance += (pose_vector[i] - pose.pose[i]) * (pose_vector[i] - pose.pose[i]);
        }
        return distance;
    }

    std::string ur2::UrPoseCombine(std::vector<double> ur_pose, double a, double v)
    {   
        using namespace std;
        std::string move_msg;
        move_msg = "movel(p[";
        move_msg = move_msg + double2string(ur_pose[0]) + ",";
        move_msg = move_msg + double2string(ur_pose[1]) + ",";
        move_msg = move_msg + double2string(ur_pose[2]) + ",";
        move_msg = move_msg + double2string(ur_pose[3]) + ",";
        move_msg = move_msg + double2string(ur_pose[4]) + ",";
        move_msg = move_msg + double2string(ur_pose[5]) + "]";
        move_msg = move_msg + ",a=";
        move_msg = move_msg + double2string(a) + ",v=";
        move_msg = move_msg + double2string(v) + ")";
        move_msg = move_msg + "\n";
        return move_msg; 
    }

    std::string ur2::double2string(double &input)
    {
        std::string string_temp;
        std::stringstream stream;
        stream<<input;
        string_temp = stream.str();
        return string_temp;
    }
}
