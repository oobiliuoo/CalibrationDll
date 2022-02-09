#pragma once
#include "AbsCalibration.h"
class EyeHandCalibration : public bl::AbsEyeHandCalibration
{
public:
	int mType();

	void handEyeCalibration();

	void handEyeCalibration(vMat imgs, vMat robot_poss, cv::Mat& H_cam2base,
		const cv::Mat cameraMatrix, const cv::Mat distCoeffs);

	void handEyeCalibration(std::string img_path_file_name, std::string robot_pos_file_name,
		cv::Mat& H_cam2base, const cv::Mat cameraMatrix, const cv::Mat distCoeffs);

private:
	void checkData(vMat imgs, vMat robot_poss, const cv::Mat cameraMatrix, const cv::Mat distCoeffs);

};

