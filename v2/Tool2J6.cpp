#include "pch.h"
#include "utils.h"
#include "BLCalibration.h"


void bl::Tool2J6(cv::Mat  j6Pos, cv::Point3d toolPoint, cv::Mat& T)
{
	// 将j6的位姿转成H
	cv::Mat H_j62base = attitudeVectorToMatrix(j6Pos, false, "xyz");
	
//	std::cout <<"j6_j6:" << j6_j6;
	cv::Mat tool_base = (cv::Mat_<double>(4, 1) << toolPoint.x, toolPoint.y, toolPoint.z, 1);

	cv::Mat tool_j6 = H_j62base.inv() * tool_base;

//	std::cout << "tool_j6:" << tool_j6;

	T = tool_j6.clone();

}