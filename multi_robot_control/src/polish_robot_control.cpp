// 打磨机器人的实际控制程序

#include "ros/ros.h"
#include "multi_robot_control/robot_control.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "iostream"
#include <tf2_ros/transform_listener.h>
#include "tf/transform_listener.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cmath"

class ROBOT_CONTROL
{
public:
    ROBOT_CONTROL()
    {
        ur_pose_vector.resize(6);
        eef_pose_vector.resize(6);
        ur_control_pub = nh.advertise<std_msgs::String>("/ur1/ur_driver/URScript", 1000);
        robot_capture_server = nh.advertiseService("robot_polish_service", &ROBOT_CONTROL::robot_polish_callback, this);
        ros::Duration(1.0).sleep();
        get_eef_pose();
    }

private:
    bool robot_polish_callback(multi_robot_control::robot_control::Request& req, multi_robot_control::robot_control::Response& res);
    void pose_to_ur_pose_vector(geometry_msgs::Pose pose, std::vector<double>& ur_pose_vector);
    void pose_to_ur_pose_vector(geometry_msgs::TransformStamped transformStamped,std::vector<double>& eef_pose_vector);
    std::string double2string(double input);
    std::string ur_pose_vector_to_control_msgs(std::vector<double> ur_pose_vector, double a, double v);
    void get_eef_pose();
    double distance(std::vector<double> target_pose, std::vector<double> current_pose);

    ros::NodeHandle nh;
    ros::ServiceServer robot_capture_server;
    ros::Publisher ur_control_pub;
    std::vector<double> ur_pose_vector;
    std::vector<double> eef_pose_vector;
    std_msgs::String ur_control_msgs;  
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "polish_robot_control");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROBOT_CONTROL robot_control;
    ros::waitForShutdown();
    return 0;
}

void ROBOT_CONTROL::get_eef_pose()
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped_ur;
    ros::Rate loop_rate(25);
    while (ros::ok())
    {
        try
        {
            // 坐标系的名称需要根据实际情况修改
            transformStamped_ur = tfBuffer.lookupTransform("base", "tool0_controller",ros::Time(0));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        pose_to_ur_pose_vector(transformStamped_ur,eef_pose_vector);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool ROBOT_CONTROL::robot_polish_callback(multi_robot_control::robot_control::Request& req, multi_robot_control::robot_control::Response& res)
{
    double a = 0.2;
    double v = 0.1;
    geometry_msgs::Pose pose = req.pose;
    pose_to_ur_pose_vector(pose,ur_pose_vector);
    ur_control_msgs.data = ur_pose_vector_to_control_msgs(ur_pose_vector,a,v);
    ur_control_pub.publish(ur_control_msgs);
    
    ros::Rate loop_rate(25);
    while (ros::ok())
    {
        // 判断当前位置和目标位姿是否相同，如果接近，则返回
        if(distance(ur_pose_vector,eef_pose_vector) < 0.005)    // 距离小于5mm，认为已经到达目标位置
        {
            res.status = true;
            return true;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ROBOT_CONTROL::pose_to_ur_pose_vector(geometry_msgs::Pose pose, std::vector<double>& ur_pose_vector)
{
    ur_pose_vector[0] = pose.position.x;
    ur_pose_vector[1] = pose.position.y;
    ur_pose_vector[2] = pose.position.z;
    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;

    Eigen::AngleAxisd axis(q);
    double theta = axis.angle();

    ur_pose_vector[3] = theta * axis.axis()[0];
    ur_pose_vector[4] = theta * axis.axis()[1];
    ur_pose_vector[5] = theta * axis.axis()[2];
}

void ROBOT_CONTROL::pose_to_ur_pose_vector(geometry_msgs::TransformStamped transformStamed,std::vector<double>& eef_pose_vector)
{
    eef_pose_vector[0] = transformStamed.transform.translation.x;
    eef_pose_vector[1] = transformStamed.transform.translation.y;
    eef_pose_vector[2] = transformStamed.transform.translation.z;

    Eigen::Quaterniond q;
    q.x() = transformStamed.transform.rotation.x;
    q.y() = transformStamed.transform.rotation.y;
    q.z() = transformStamed.transform.rotation.z;
    q.w() = transformStamed.transform.rotation.w;

    Eigen::AngleAxisd axis(q);
    double theta = axis.angle();

    eef_pose_vector[3] = theta * axis.axis()[0];
    eef_pose_vector[4] = theta * axis.axis()[1];
    eef_pose_vector[5] = theta * axis.axis()[2];
}

std::string ROBOT_CONTROL::ur_pose_vector_to_control_msgs(std::vector<double> ur_pose, double a, double v)
{
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

std::string ROBOT_CONTROL::double2string(double input)
{
    std::string string_temp;
    std::stringstream stream;
    stream<<input;
    string_temp = stream.str();
    return string_temp;
}

double ROBOT_CONTROL::distance(std::vector<double> target_pose, std::vector<double> current_pose)
{
    double distance = (target_pose[0] - current_pose[0]) * (target_pose[0] - current_pose[0]) 
                    + (target_pose[1] - current_pose[1]) * (target_pose[1] - current_pose[1]) 
                    + (target_pose[2] - current_pose[2]) * (target_pose[2] - current_pose[2]) ;
    return sqrt(distance);
}

