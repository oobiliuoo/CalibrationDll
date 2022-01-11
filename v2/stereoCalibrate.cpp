#include "pch.h"
#include "BLCalibration.h"

// 检测棋盘格内角点在图像中坐标的函数
void getImgsPoints(bl::vMat imgs, std::vector<std::vector<cv::Point2f>>& Points, cv::Size boardSize)
{
	for (int i = 0; i < imgs.size(); i++)
	{
		cv::Mat img1 = imgs[i];
		cv::Mat gray1;
		cv::cvtColor(img1, gray1, CV_BGR2GRAY);
		std::vector<cv::Point2f> img1_points;
		cv::findChessboardCornersSB(gray1,boardSize,img1_points, cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
		Points.push_back(img1_points);
	}

}

void bl::stereoCalibrate(std::string img_1_path_file_name, std::string img_2_path_file_name,
	const cv::Size& board_size,const cv::Size& square_size,
	bl::BSC& bsc)
{
	vMat imgLs;
	vMat imgRs;
	std::string imgLName;
	std::string imgRName;
	std::ifstream finL(img_1_path_file_name);
	std::ifstream finR(img_2_path_file_name);

	while (getline(finL, imgLName) && getline(finR, imgRName))
	{

		cv::Mat imgL = cv::imread(imgLName);
		cv::Mat imgR = cv::imread(imgRName);
		int i = 0;
		if (!imgL.data && !imgR.data)
		{
			std::cout << "check the img: "<<i << std::endl;
			continue ;
		}
		imgLs.push_back(imgL);
		imgRs.push_back(imgR);
		i++;

	}

	std::vector<std::vector<cv::Point2f>> imgLsPoints;
	std::vector<std::vector<cv::Point2f>> imgRsPoints;

	getImgsPoints(imgLs, imgLsPoints, board_size);
	getImgsPoints(imgRs, imgRsPoints, board_size);

	std::vector<std::vector<cv::Point3f>> objectPoints;
	
	for (int i = 0; i < imgLsPoints.size(); i++)
	{
		std::vector<cv::Point3f> tempPointSet;
		for (int j = 0; j < board_size.height; j++)
		{
			for (int k = 0; k < board_size.width; k++)
			{
				cv::Point3f realPoint;
				realPoint.x = j * square_size.width;
				realPoint.y = k * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}

		}
		objectPoints.push_back(tempPointSet);
	}

	cv::Size imageSize;
	imageSize.width = imgLs[0].cols;
	imageSize.height = imgLs[0].rows;


	cv::Mat rvecs, tvecs;
	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, DBL_EPSILON);
	cv::calibrateCamera(objectPoints,imgLsPoints,imageSize,bsc._left_cameraMatrix,bsc._left_distCoeffs,rvecs,tvecs,0,criteria);
	cv::calibrateCamera(objectPoints,imgRsPoints,imageSize,bsc._right_cameraMatrix,bsc._right_distCoeffs,rvecs,tvecs,0,criteria);
	
	cv::Mat R, T;
	cv::stereoCalibrate(objectPoints, imgLsPoints, imgRsPoints, bsc._left_cameraMatrix, bsc._left_distCoeffs,
		bsc._right_cameraMatrix, bsc._right_distCoeffs, imageSize,
		R, T,
		bsc._E, bsc._F,
		cv::CALIB_USE_INTRINSIC_GUESS
	//	cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5
	);

	bl::R_T2H(R, T, bsc._H_l2r);



	//std::cout << "the H:\n" << bsc._H_l2r << std::endl;

}