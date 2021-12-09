#include "pch.h"
#include "BLCalibration.h"

void bl::H2R_T(cv::Mat H, cv::Mat& R, cv::Mat& T)
{
	cv::Rect R_rect(0, 0, 3, 3);
	cv::Rect T_rect(3, 0, 1, 3);
	R = H(R_rect);
	T = H(T_rect);

}