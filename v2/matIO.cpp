#include "pch.h"
#include "BLCalibration.h"
#include "utils.h"

void writeMat(std::string file, cv::Mat src, std::string name)
{
	cv::FileStorage fs(file, cv::FileStorage::APPEND);
	fs << name << src;
	fs.release();
}

void readMat(std::string file, cv::Mat& dst, std::string name)
{
	cv::FileStorage fs(file, cv::FileStorage::READ);
	fs[name] >> dst;
	fs.release();
}