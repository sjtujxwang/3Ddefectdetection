#include "ros/ros.h"
#include "HalconCpp.h"
#include "multi_robot_control/basler_detection.h"
#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>  

using namespace HalconCpp;

bool basler_detection_callback(multi_robot_control::basler_detection::Request& req, multi_robot_control::basler_detection::Response& res)
{
     res.poses.resize(1);
	 HObject  ho_Image, ho_GrayImage, ho_Regions, ho_ConnectedRegions;
	 HObject  ho_SelectedRegions, ho_SelectedRegions1, ho_SelectedRegions2;

	// // Local control variables
	 HTuple  hv_AcqHandle, hv_Area, hv_Row, hv_Column;

     OpenFramegrabber("GigEVision2", 0, 0, 0, 0, 0, 0, "progressive", -1, "default", 
       -1, "false", "default", "camera", 0, -1, &hv_AcqHandle);
	 GrabImageStart(hv_AcqHandle, -1);

	 GrabImageAsync(&ho_Image, hv_AcqHandle, -1);
	 //Image Acquisition 01: Do something
	 CloseFramegrabber(hv_AcqHandle);

	 Rgb1ToGray(ho_Image, &ho_GrayImage);
	 Threshold(ho_GrayImage, &ho_Regions, 0, 17);
	 Connection(ho_Regions, &ho_ConnectedRegions);
	 SelectShape(ho_ConnectedRegions, &ho_SelectedRegions, "area", "and", 0, 117188);
	 SelectShape(ho_SelectedRegions, &ho_SelectedRegions1, "area", "and", 341, 625.26);
	 SelectShape(ho_SelectedRegions1, &ho_SelectedRegions2, (HTuple("area").Append("row")),"and",(HTuple(400).Append(2025.92)),(HTuple(555).Append(3163)));
	 AreaCenter(ho_SelectedRegions2, &hv_Area, &hv_Row, &hv_Column);

	double r, c;
    // r=2424;
    // c=2995;
	 r = double(hv_Row);
	 c = double(hv_Column);
	 std::cout << "Row= " << r << ",Column= " << c << std::endl;


    Eigen::Matrix<float, 3, 4> projection_matrix;
    projection_matrix<<3787.171875, 0.000000, 2481.716654, 0.000000, 0.000000, 3862.765625, 1914.001282, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000;
    std::cout << "projection_matrix:\n" << projection_matrix << std::endl;

    float zc= 1000;
    Eigen::Matrix<float, 4, 3> projection_matrix_inv;
    projection_matrix_inv(0,0) = zc/projection_matrix(0,0) ;
    projection_matrix_inv(0,1) = 0.0 ;
    projection_matrix_inv(0,2) = -zc*projection_matrix(0,2)/projection_matrix(0,0) ;

    projection_matrix_inv(1,0) =  0.0 ;
    projection_matrix_inv(1,1) =  zc/projection_matrix(1,1) ;
    projection_matrix_inv(1,2) =  -zc*projection_matrix(1,2)/projection_matrix(1,2) ;

    projection_matrix_inv(2,0) = 0.0;
    projection_matrix_inv(2,1) = 0.0;
    projection_matrix_inv(2,2) = zc;

    projection_matrix_inv(3,0) = 0.0;
    projection_matrix_inv(3,1) =0.0;
    projection_matrix_inv(3,2) = 1.0;

    std::cout << "projection_matrix_inv:\n" << projection_matrix_inv << std::endl;



    Eigen::Matrix<float, 3, 1> UV1;
    UV1<<r, c, 1.0;

    Eigen::Matrix<float, 4, 1> XYZ1;

    XYZ1 = projection_matrix_inv*UV1;
    std::cout << "XYZ1:\n" << XYZ1 << std::endl;


    res.poses[0].position.x = 0.118;
    res.poses[0].position.y = -0.979;     
    res.poses[0].position.z = 0.505;
    res.poses[0].orientation.x = 0.532;
    res.poses[0].orientation.y = 0.846;
    res.poses[0].orientation.z = 0.003;
    res.poses[0].orientation.w = 0.002;
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "basler_detection");
    ros::NodeHandle nh;
    ros::ServiceServer basler_detection_server = nh.advertiseService("basler_detection_service", basler_detection_callback);
    ROS_INFO("Ready to process image by basler!");
    ros::spin();
    return 0;
}