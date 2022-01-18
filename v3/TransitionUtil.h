#pragma once
#include "AbsUtils.h"
class CLASS_DECLSPEC TransitionUtil : public bl::AbsTransUtils
{
public:
	void H2R_T(cv::Mat H, cv::Mat& R, cv::Mat& T);
	void R_T2H(cv::Mat R, cv::Mat T, cv::Mat& H);
	void to_CV_32F(cv::Mat& src);

	bool isRotatedMatrix(cv::Mat& R);

	cv::Mat eulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const std::string& seq);
	cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q);
	cv::Mat attitudeVectorToMatrix(const cv::Mat& m, bool useQuaternion, const std::string& seq);

};

