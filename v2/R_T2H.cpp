#include "pch.h"
#include "BLCalibration.h"

void bl::R_T2H(cv::Mat R,cv::Mat T,cv::Mat& H)
{
	// CV_64F
	if (R.type()==6) {
		cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) <<
			R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
			0, 0, 0);
		cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) <<
			T.at<double>(0, 0),
			T.at<double>(1, 0),
			T.at<double>(2, 0),
			1);
		cv::hconcat(R1, T1, H);		//æÿ’Û∆¥Ω”
	}
	else
	{
		cv::Mat_<float> R1 = (cv::Mat_<float>(4, 3) <<
			R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
			R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
			R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2),
			0, 0, 0);
		cv::Mat_<float> T1 = (cv::Mat_<float>(4, 1) <<
			T.at<float>(0, 0),
			T.at<float>(1, 0),
			T.at<float>(2, 0),
			1);

		cv::hconcat(R1, T1, H);		//æÿ’Û∆¥Ω”
	}


}