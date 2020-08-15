// 该程序负责将两次手眼标定的结果输入到程序中，然后求解两个机器人之间的坐标变换
#include "ros/ros.h"
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2/convert.h"
#include "geometry_msgs/Pose.h"
#include "iostream"
#include "iomanip"
#include "cmath"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "realsense_frame_transform");
    ros::NodeHandle nh;

    geometry_msgs::Pose optical_frame_to_camera_link, optical_frame_to_tool0, camera_link_to_tool0;

    optical_frame_to_camera_link.position.x = optical_frame_to_camera_link.position.y = optical_frame_to_camera_link.position.z = 0;
    optical_frame_to_camera_link.orientation.x = -0.5;
    optical_frame_to_camera_link.orientation.y = 0.5;
    optical_frame_to_camera_link.orientation.z = -0.5;
    optical_frame_to_camera_link.orientation.w = 0.5;

    optical_frame_to_tool0.position.x = 0.1335103624464409;
    optical_frame_to_tool0.position.y = -0.01313475243054544;
    optical_frame_to_tool0.position.z = 0.1425974729618332;
    optical_frame_to_tool0.orientation.x = -0.1009224551911633;
    optical_frame_to_tool0.orientation.y = -0.09745011586088743;
    optical_frame_to_tool0.orientation.z = 0.6833620361361891;
    optical_frame_to_tool0.orientation.w = 0.7164736286317097;

    Eigen::Affine3d optical_frame_to_camera_link_matrix, optical_frame_to_tool0_matrix, camera_link_to_tool0_matrix;

    tf2::convert(optical_frame_to_camera_link, optical_frame_to_camera_link_matrix);
    tf2::convert(optical_frame_to_tool0, optical_frame_to_tool0_matrix);

    camera_link_to_tool0_matrix = optical_frame_to_tool0_matrix * optical_frame_to_camera_link_matrix.inverse();
    tf2::convert(camera_link_to_tool0_matrix, camera_link_to_tool0);

    ROS_INFO_STREAM("Camera link to tool0 is: "<<camera_link_to_tool0);




    // Eigen::Affine3d camera_to_base1_matrix, camera_to_base2_matrix, base1_to_base2_matrix, base_to_base_link_matrix;

    // base_to_base_link.position.x = base_to_base_link.position.y = base_to_base_link.position.z = 0;
    // base_to_base_link.orientation.x = base_to_base_link.orientation.y = base_to_base_link.orientation.w = 0;
    // base_to_base_link.orientation.z = 1;
    

    // camera_to_base1.position.x = -0.09955895076410565;
    // camera_to_base1.position.y = -1.3828071581532893;
    // camera_to_base1.position.z = 1.1017535288819797;
    // camera_to_base1.orientation.x = 0.025711392457025174;
    // camera_to_base1.orientation.y = 0.9973543463037003;
    // camera_to_base1.orientation.z = 0.06268333327193465;
    // camera_to_base1.orientation.w = 0.02634448589285042;

    // camera_to_base2.position.x = 0.13558333050663193;
    // camera_to_base2.position.y = -0.5946130643935611;
    // camera_to_base2.position.z = 1.0679535793891912;
    // camera_to_base2.orientation.x = 0.9969591591579152;
    // camera_to_base2.orientation.y = -0.050108228639211654;
    // camera_to_base2.orientation.z = -0.03298392816018064;
    // camera_to_base2.orientation.w = 0.0497359113408739;

    // tf2::convert(base_to_base_link, base_to_base_link_matrix);

    // tf2::convert(camera_to_base1, camera_to_base1_matrix);
    // camera_to_base1_matrix = base_to_base_link_matrix * camera_to_base1_matrix;

    // tf2::convert(camera_to_base2, camera_to_base2_matrix);
    // camera_to_base2_matrix = base_to_base_link_matrix * camera_to_base2_matrix;

    // base1_to_base2_matrix = camera_to_base1_matrix * camera_to_base2_matrix.inverse();

    // Eigen::Matrix3d rotation_matrix;
    // rotation_matrix << base1_to_base2_matrix(0,0), base1_to_base2_matrix(0,1), base1_to_base2_matrix(0,2), 
    //                    base1_to_base2_matrix(1,0), base1_to_base2_matrix(1,1), base1_to_base2_matrix(1,2),
    //                    base1_to_base2_matrix(2,0), base1_to_base2_matrix(2,1), base1_to_base2_matrix(2,2);
    // Eigen::Vector3d euler_angle = rotation_matrix.eulerAngles(2,1,0);

    // using std::cout;
    // using std::endl;
    // using std::setw;

    // int tab = 20;

    // cout<<setw(tab)<<"The transformation between two ur robot is"<<endl;
    // cout<<setw(tab)<<"translation_x"<<setw(tab)<<base1_to_base2_matrix(0,3)<<endl;
    // cout<<setw(tab)<<"translation_y"<<setw(tab)<<base1_to_base2_matrix(1,3)<<endl;
    // cout<<setw(tab)<<"translation_z"<<setw(tab)<<base1_to_base2_matrix(2,3)<<endl;
    // cout<<setw(tab)<<"roll"<<setw(tab)<<euler_angle(2)<<endl;
    // cout<<setw(tab)<<"pitch"<<setw(tab)<<euler_angle(1)<<endl;
    // cout<<setw(tab)<<"yaw"<<setw(tab)<<euler_angle(0)<<endl;


    // double distance = sqrt(base1_to_base2_matrix(0,3) * base1_to_base2_matrix(0,3) + base1_to_base2_matrix(1,3) * base1_to_base2_matrix(1,3));

    // ROS_INFO_STREAM("Distance between two robot is "<<distance);

    return 0;
}