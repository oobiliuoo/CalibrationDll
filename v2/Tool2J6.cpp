#include "pch.h"
#include "utils.h"
#include "BLCalibration.h"


void bl::Tool2J6(cv::Mat  j6Pos, cv::Point3d toolPoint, cv::Mat& H)
{
	// 将j6的位姿转成H
	cv::Mat H_j62base = attitudeVectorToMatrix(j6Pos, false, "xyz");
	cv::Mat R_j6, T_j6;
	H2R_T(H_j62base,R_j6,T_j6);

	cv::Mat tool_base = (cv::Mat_<double>(4, 1) << toolPoint.x, toolPoint.y, toolPoint.z, 1);
	cv::Mat H_tool2base, H_tool2j6;
	R_T2H(R_j6, tool_base,H_tool2base);


	H_tool2j6 = H_j62base.inv() * H_tool2base;

	H = H_tool2j6.clone();

}