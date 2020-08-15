//C++
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

//ROS
#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2/convert.h"

//eigrn
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Core"

//OpenCV
#include <opencv2/opencv.hpp>

//boost
#include <boost/thread/thread.hpp>

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

//Customed
#include "gocator3200.h"
#include "config.h"
#include "pointcloud_helper.h"
#include "dbscan.h"
#include "DefectDetect.h"
#include "multi_robot_control/user_input.h"
#include "multi_robot_control/basler_detection.h"
#include "multi_robot_control/robot_control.h"
#include "multi_robot_control/polish.h"


using namespace std;
using namespace cv;

#ifndef PI
#define PI 3.1415926535897932384626433832795028841971693993751058209749445
#endif
#ifndef pi
#define pi 3.1415926535897932384626433832795028841971693993751058209749445
#endif

class MAIN_NODE
{
public:
    MAIN_NODE()
    {
        ros::param::get("basler_to_capture_robot/x", basler_to_capture_robot.position.x);
        ros::param::get("basler_to_capture_robot/y", basler_to_capture_robot.position.y);
        ros::param::get("basler_to_capture_robot/z", basler_to_capture_robot.position.z);
        ros::param::get("basler_to_capture_robot/qx", basler_to_capture_robot.orientation.x);
        ros::param::get("basler_to_capture_robot/qy", basler_to_capture_robot.orientation.y);
        ros::param::get("basler_to_capture_robot/qz", basler_to_capture_robot.orientation.z);
        ros::param::get("basler_to_capture_robot/qw", basler_to_capture_robot.orientation.w);

        ros::param::get("basler_to_polish_robot/x", basler_to_polish_robot.position.x);
        ros::param::get("basler_to_polish_robot/y", basler_to_polish_robot.position.y);
        ros::param::get("basler_to_polish_robot/z", basler_to_polish_robot.position.z);
        ros::param::get("basler_to_polish_robot/qx", basler_to_polish_robot.orientation.x);
        ros::param::get("basler_to_polish_robot/qy", basler_to_polish_robot.orientation.y);
        ros::param::get("basler_to_polish_robot/qz", basler_to_polish_robot.orientation.z);
        ros::param::get("basler_to_polish_robot/qw", basler_to_polish_robot.orientation.w);

        ros::param::get("gocator_to_capture_robot_eef/x", gocator_to_capture_robot_eef.position.x);
        ros::param::get("gocator_to_capture_robot_eef/y", gocator_to_capture_robot_eef.position.y);
        ros::param::get("gocator_to_capture_robot_eef/z", gocator_to_capture_robot_eef.position.z);
        ros::param::get("gocator_to_capture_robot_eef/qx", gocator_to_capture_robot_eef.orientation.x);
        ros::param::get("gocator_to_capture_robot_eef/qy", gocator_to_capture_robot_eef.orientation.y);
        ros::param::get("gocator_to_capture_robot_eef/qz", gocator_to_capture_robot_eef.orientation.z);
        ros::param::get("gocator_to_capture_robot_eef/qw", gocator_to_capture_robot_eef.orientation.w);

        ROS_INFO_STREAM("gocator_to_capture_robot_eef"<<gocator_to_capture_robot_eef);
        ROS_INFO_STREAM("basler_to_polish_robot"<<basler_to_polish_robot);
        ROS_INFO_STREAM("basler_to_capture_robot"<<basler_to_capture_robot);


        // 下面的两个pose需要根据实际的工况调整handeye_calibration.yaml
        ros::param::get("polish_robot_initial_pose/x", robot_polish_reset_pose.position.x);
        ros::param::get("polish_robot_initial_pose/y", robot_polish_reset_pose.position.y);
        ros::param::get("polish_robot_initial_pose/z", robot_polish_reset_pose.position.z);
        ros::param::get("polish_robot_initial_pose/qx", robot_polish_reset_pose.orientation.x);
        ros::param::get("polish_robot_initial_pose/qy", robot_polish_reset_pose.orientation.y);
        ros::param::get("polish_robot_initial_pose/qz", robot_polish_reset_pose.orientation.z);
        ros::param::get("polish_robot_initial_pose/qw", robot_polish_reset_pose.orientation.w);

        ros::param::get("capture_robot_initial_pose/x", robot_capture_reset_pose.position.x);
        ros::param::get("capture_robot_initial_pose/y", robot_capture_reset_pose.position.y);
        ros::param::get("capture_robot_initial_pose/z", robot_capture_reset_pose.position.z);
        ros::param::get("capture_robot_initial_pose/qx", robot_capture_reset_pose.orientation.x);
        ros::param::get("capture_robot_initial_pose/qy", robot_capture_reset_pose.orientation.y);
        ros::param::get("capture_robot_initial_pose/qz", robot_capture_reset_pose.orientation.z);
        ros::param::get("capture_robot_initial_pose/qw", robot_capture_reset_pose.orientation.w);


        capture_robot_pose_sub = nh_.subscribe("capture_robot_tool_pose", 1000, &MAIN_NODE::CaptureRobotPoseCallback, this);
        basler_detection_client = nh_.serviceClient<multi_robot_control::basler_detection>("basler_detection_service");
        robot_capture_client = nh_.serviceClient<multi_robot_control::robot_control>("robot_capture_service");
        robot_polish_client = nh_.serviceClient<multi_robot_control::robot_control>("robot_polish_service");
        polish_client = nh_.serviceClient<multi_robot_control::polish>("polish_service");
        user_input_server = nh_.advertiseService("user_input_service", &MAIN_NODE::user_input_callback, this);
        device.setIP("192.168.1.10");
    }

private:

    // function
    bool user_input_callback(multi_robot_control::user_input::Request& req, multi_robot_control::user_input::Response& res);
    void object_in_basler_to_capture_robot(geometry_msgs::Pose input_pose, geometry_msgs::Pose& output_pose);
    void object_gocator_to_polish_robot(geometry_msgs::Pose input_pose, geometry_msgs::Pose& output_pose);
    void CaptureRobotPoseCallback(const geometry_msgs::PoseConstPtr& pose_msg);

    ros::NodeHandle nh_;
    ros::ServiceServer user_input_server;
    ros::ServiceClient basler_detection_client;
    ros::ServiceClient robot_capture_client;
    ros::ServiceClient robot_polish_client;
    ros::ServiceClient polish_client;

    multi_robot_control::basler_detection basler_detection_srv;
    multi_robot_control::robot_control robot_capture_srv;
    multi_robot_control::robot_control robot_polish_srv;
    multi_robot_control::polish polish_srv;

    ros::Subscriber capture_robot_pose_sub;     // 接收capture robot末端的位姿信息
        

    geometry_msgs::Pose robot_capture_reset_pose, robot_polish_reset_pose;
    geometry_msgs::Pose basler_to_capture_robot, basler_to_polish_robot, gocator_to_capture_robot_eef;     // 三个手眼标定结果
    geometry_msgs::Pose capture_robot_eef_to_base;  // 需要获取拍照机器人的末端相对基座的位姿

    Gocator3200::Device device;
};

int main(int argc, char *argv[])
{
    std::string pkg_loc = ros::package::getPath("multi_robot_control");
    Config::setParameterFile(pkg_loc+"/parameters/parameter.yml");
    ros::init(argc, argv, "main_node");
    ros::AsyncSpinner spinner(5);
    spinner.start();
    MAIN_NODE main_node;
    ros::waitForShutdown();
    return 0;
}

bool MAIN_NODE::user_input_callback(multi_robot_control::user_input::Request& req, multi_robot_control::user_input::Response& res)
{
    // 0. 两个机械臂应该复位
    robot_polish_srv.request.pose = robot_polish_reset_pose;
    ros::service::waitForService("robot_polish_service");
    robot_polish_client.call(robot_polish_srv);
    ros::Duration(1.0).sleep();
    ROS_INFO("Polish robot has been to initial pose!");

    robot_capture_srv.request.pose = robot_capture_reset_pose;
    ros::service::waitForService("robot_capture_service");
    robot_capture_client.call(robot_capture_srv);
    ros::Duration(1.0).sleep();
    ROS_INFO("Capture robot has been to initial pose!");


    // 1. basler相机检测
    basler_detection_srv.request.capture = true;
    ros::service::waitForService("basler_detection_service");
    basler_detection_client.call(basler_detection_srv);

    int defect_number = basler_detection_srv.response.poses.size();
    if(defect_number == 0)
    {
        ROS_ERROR("There is no defect to polish!");
        res.status = true;
        return true;
    }
    else
    {
        ROS_INFO_STREAM("There all "<<defect_number<<" defects!");
        // 有多个缺陷需要按照顺序打磨
        for(int i = 0; i < defect_number; i ++)
        {
            // 2. 检测机械臂就位
            // object_in_basler_to_capture_robot(basler_detection_srv.response.poses[i],robot_capture_srv.request.pose);
            // robot_capture_client.call(robot_capture_srv);

            // 下面的两行只在测试时使用
            robot_capture_srv.request.pose = basler_detection_srv.response.poses[i];
            robot_capture_client.call(robot_capture_srv);
            ROS_INFO("Capture robot has been to target position!");
            ros::Duration(1.0).sleep();

            // 3. gocator相机检测，不使用service的形式，直接使用库函数调用
	        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);   // zhangheng 3
            device.getSingleSnapshot(*cloud);
            std::cout<<"point cloud size:"<<cloud->points.size()<<std::endl;


            // //可视化
            // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud"));
            // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloud, 0, 255, 0);
            // viewer->addPointCloud(cloud, "cloud");
            // viewer->addCoordinateSystem(10);
            // while(!viewer->wasStopped())
            // {
            //     viewer->spinOnce(100);
            //     boost::this_thread::sleep(boost::posix_time::microseconds(100));
            // }
            // viewer->updatePointCloud(cloud, "cloud");
            // viewer->removePointCloud("cloud");
            // viewer->removeAllPointClouds();
            // viewer->close();
            // device.stop();

            std::vector<float> res = process(cloud);   
            for (int i = 0; i < res.size(); ++i)
            {
                std::cout<<res[i]<<std::endl;
            }


            Eigen::Vector3d NZ(0.0, 0.0, 1.0);

            std::cout<<"\n******************************************************************************************************"<<std::endl;
            Eigen::Vector3d tmp_NZ;
            if(res[5] <0)
            {
                res[3]=-res[3];
                res[4]=-res[4];
                res[5]=-res[5];
            }
            tmp_NZ(0) = res[3];
            tmp_NZ(1) = res[4];
            tmp_NZ(2) = res[5];

            std::cout<<"tmp_NZ:\n"<<tmp_NZ<<std::endl;
            tmp_NZ.normalize();
            std::cout<<"\ntmp_NZ (normalized):\n"<<tmp_NZ<<std::endl;
            double angle = acos(NZ.dot(tmp_NZ));
            Eigen::Vector3d axis = NZ.cross(tmp_NZ);
            axis.normalize();
            Eigen::Vector3d axisangle = axis*angle;

            std::cout
            << "\nNZ:\n"<< NZ <<"\n"
            << "\ntmp_NZ:\n"	<< tmp_NZ <<"\n"
            << "\ndot:\n"<< NZ.dot(tmp_NZ) <<"\n"
            << "\nangle     : " << angle
            << "\nangle[deg]: " << angle*180/3.1415926 <<"\n"
            << "\naxis (normalized):\n" << axis<<"\n"
            << "\naxisangle:\n"<<axisangle<<"\n"
            <<std::endl;

            Eigen::AngleAxisd rotation_vector(angle,axis);
            Eigen::Quaterniond quaternion(rotation_vector);
            ROS_INFO("3D detection has been finished!");
	        // getchar();


            // 转换到打磨机器人位姿下面去
            geometry_msgs::Pose object_in_gocator;         
            object_in_gocator.position.x = res[0]/1000;
            object_in_gocator.position.y = res[1]/1000;
            object_in_gocator.position.z = res[2]/1000;
            object_in_gocator.orientation.x = quaternion.x();
            object_in_gocator.orientation.y = quaternion.y();
            object_in_gocator.orientation.z = quaternion.z();
            object_in_gocator.orientation.w = quaternion.w();

            // geometry_msgs::Pose object_in_gocator; 
            // object_in_gocator.position.x = 0;
            // object_in_gocator.position.y = 0;
            // object_in_gocator.position.z = 0.3;     // 最大330
            // object_in_gocator.orientation.x = 0;
            // object_in_gocator.orientation.y = 0;
            // object_in_gocator.orientation.z = 0;
            // object_in_gocator.orientation.w = 1;

            getchar();
            object_gocator_to_polish_robot(object_in_gocator,robot_polish_srv.request.pose);
            std::cout
            <<robot_polish_srv.request.pose.position.x<<"\n"
            <<robot_polish_srv.request.pose.position.y<<"\n"
            <<robot_polish_srv.request.pose.position.z<<"\n"
            <<std::endl;


            // 4. 检测机械臂复位
            robot_capture_srv.request.pose = robot_capture_reset_pose;
            robot_capture_client.call(robot_capture_srv);
            ros::Duration(1.0).sleep();
            ROS_INFO("Capture robot has been to initial pose!");



            // 5. 打磨机器人就位
            robot_polish_srv.request.pose.position.z += 0.1;
            robot_polish_client.call(robot_polish_srv);
            robot_polish_srv.request.pose.position.z -= 0.07;
            robot_polish_client.call(robot_polish_srv);
            ros::Duration(1.0).sleep();
            ROS_INFO("Polish robot has been to target pose!");      // 注意打磨机器人的末端长度

            // 6. 打磨头开始打磨
            // polish_srv.request.start = true;
            // ros::service::waitForService("polish_service");
            // polish_client.call(polish_srv);
            // ROS_INFO("The defect has been polished!");
            ros::Duration(5.0).sleep();

            // // 7. 打磨机器人复位
            // robot_polish_srv.request.pose = robot_polish_reset_pose;
            // robot_polish_client.call(robot_polish_srv);
            // ros::Duration(1.0).sleep();
            // ROS_INFO("Polish robot has been to initial pose!");
        }
        ROS_INFO("All defects have been polished!");
        res.status = true;
        return true; 
    }
}

void MAIN_NODE::object_in_basler_to_capture_robot(geometry_msgs::Pose input_pose, geometry_msgs::Pose& output_pose)
{
    Eigen::Affine3d basler_to_capture_robot_matrix, input_pose_matrix, output_pose_matrix;
    tf2::convert(input_pose,input_pose_matrix);
    tf2::convert(basler_to_capture_robot,basler_to_capture_robot_matrix);
    output_pose_matrix = basler_to_capture_robot_matrix * input_pose_matrix;
    tf2::convert(output_pose_matrix,output_pose);
}

void MAIN_NODE::object_gocator_to_polish_robot(geometry_msgs::Pose input_pose, geometry_msgs::Pose& output_pose)
{
    // 需要根据实际的标定情况进行调整

    Eigen::Affine3d input_pose_matrix, gocator_to_capture_robot_eef_matrix, capture_robot_eef_to_base_matrix, basler_to_capture_robot_matrix, basler_to_polish_robot_matrix, output_pose_matrix;
    tf2::convert(input_pose, input_pose_matrix);
    tf2::convert(gocator_to_capture_robot_eef, gocator_to_capture_robot_eef_matrix);
    tf2::convert(capture_robot_eef_to_base, capture_robot_eef_to_base_matrix);
    tf2::convert(basler_to_capture_robot, basler_to_capture_robot_matrix);
    tf2::convert(basler_to_polish_robot, basler_to_polish_robot_matrix);
    
    output_pose_matrix = basler_to_polish_robot_matrix * basler_to_capture_robot_matrix.inverse() * capture_robot_eef_to_base_matrix * gocator_to_capture_robot_eef_matrix * input_pose_matrix;
    tf2::convert(output_pose_matrix,output_pose);
}

void MAIN_NODE::CaptureRobotPoseCallback(const geometry_msgs::PoseConstPtr& pose_msg)
{
    capture_robot_eef_to_base = *pose_msg;
}