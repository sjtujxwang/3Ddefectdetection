#include "ros/ros.h"
#include "HalconCpp.h"
#include <stdio.h>


int main(int argc, char *argv[])
{
    using namespace HalconCpp;
    ros::init(argc, argv, "halcon_ros_test");
    ros::NodeHandle nh;


	// Local iconic variables
	HObject  ho_Image, ho_Image1, ho_Image2, ho_Image3;
	HObject  ho_ImageInvert, ho_Regions, ho_ConnectedRegions;
	HObject  ho_SelectedRegions;

	// Local control variables
	HTuple  hv_Width, hv_Height, hv_Area, hv_Row;
	HTuple  hv_Column;


	ReadImage(&ho_Image, "/home/leon/caoqi/src/halcon_ros_test/pvc.jpg");
	GetImageSize(ho_Image, &hv_Width, &hv_Height);

    ROS_INFO("Hello1");

	Decompose3(ho_Image, &ho_Image1, &ho_Image2, &ho_Image3);
	InvertImage(ho_Image1, &ho_ImageInvert);
	Threshold(ho_ImageInvert, &ho_Regions, 76, 111);

    ROS_INFO("Hello2");
	Connection(ho_Regions, &ho_ConnectedRegions);
	SelectShape(ho_ConnectedRegions, &ho_SelectedRegions, (HTuple("area").Append("row")),
		"and", (HTuple(1756.33).Append(100.74)), (HTuple(10000).Append(305.38)));
	AreaCenter(ho_SelectedRegions, &hv_Area, &hv_Row, &hv_Column);
    ROS_INFO("Hello3");
	// double r, c;
	// r = double(hv_Row);
	// c = double(hv_Column);
    // ROS_INFO("Hello4");
    auto length = hv_Area.Length();
	std::cout << "Row= " << length << ",Column= " << length << std::endl;
    
	system("pause");
    
    return 0;
}