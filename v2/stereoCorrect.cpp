#include "pch.h"
#include "BLCalibration.h"

void bl::stereoCorrect(BSC& bsc, cv::Mat& imgl, cv::Mat& imgr, cv::Rect* validRoi)
{


	cv::Size imageSize;
	imageSize.width = imgl.cols;
	imageSize.height = imgl.rows;

	cv::Mat R, T;
	bl::H2R_T(bsc._H_l2r, R, T);

	stereoRectify(bsc._left_cameraMatrix, bsc._left_distCoeffs,
		bsc._right_cameraMatrix, bsc._right_distCoeffs,
		imageSize, R, T, bsc._R1, bsc._R2, bsc._P1, bsc._P2, bsc._Q,
		cv::CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);


	std::cout << "Q;\n" << bsc._Q << std::endl;

	// 计算矫正投影矩阵
	cv::Mat rmap[2][2];
	cv::initUndistortRectifyMap(bsc._left_cameraMatrix,bsc._left_distCoeffs,bsc._R1,bsc._P1,imageSize,CV_16SC2,rmap[0][0],rmap[0][1]);
	cv::initUndistortRectifyMap(bsc._right_cameraMatrix,bsc._right_distCoeffs,bsc._R2,bsc._P2,imageSize,CV_16SC2,rmap[1][0],rmap[1][1]);


	cv::remap(imgl, imgl, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
	cv::remap(imgr, imgr, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);

	int width = validRoi[0].width > validRoi[1].width ? validRoi[0].width : validRoi[1].width;
	int height = validRoi[0].height > validRoi[1].height ? validRoi[0].height : validRoi[1].height;

	imgl = imgl(cv::Rect(validRoi[0].x, validRoi[0].y, width, height));
	imgr = imgr(cv::Rect(validRoi[1].x, validRoi[1].y, width, height));

}