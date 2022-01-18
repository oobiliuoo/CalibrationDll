#pragma once
#include "AbsCalibration.h"

class CLASS_DECLSPEC CamCalibration : public bl::AbsCamCalibration
{

public:
	CamCalibration() {};

	~CamCalibration(){}
	// 单目相机标定
	int monocularCalbration(std::string img_path_file_name, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		cv::Size board_size = cv::Size(11, 8), cv::Size square_size = cv::Size(5, 5));

	int mType();

};

