#include "pch.h"
#include "BLCalibration.h"

void bl::getImgRT(const cv::Mat srcImg, cv::Mat& R, cv::Mat& T, const cv::Mat cameraMatrix, const cv::Mat distCoeffs)
{
	std::vector<cv::Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
	cv::Size board_size = cv::Size(11, 8);    /* 标定板上每行、列的角点数 */
	/*棋盘三维信息*/
	cv::Size square_size = cv::Size(5, 5);  /* 实际测量得到的标定板上每个棋盘格的大小 */
		/* 提取角点 */
	if (0 == findChessboardCornersSB(srcImg, board_size, image_points_buf,  cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY))
	{
		std::cout << "can not find chessboard corners!\n"; //找不到角点
		return ;
	}
	//else
	//{
	//	cv::Mat view_gray;
	//	cvtColor(srcImg, view_gray, CV_RGB2GRAY);
	//	/* 亚像素精确化 */
	//	find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(11, 11)); //对粗提取的角点进行精确化
	//
	//}
	/* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	std::vector<cv::Point3f> object_points;
	for (i = 0; i < board_size.height; i++)
	{
		for (j = 0; j < board_size.width; j++)
		{
			cv::Point3f realPoint;
			/* 假设标定板放在世界坐标系中z=0的平面上 */
			realPoint.x = i * square_size.width;
			realPoint.y = j * square_size.height;
			realPoint.z = 0;
			object_points.push_back(realPoint);
		}
	}

	// 获取rt矩阵
	cv::solvePnPRansac(object_points, image_points_buf, cameraMatrix, distCoeffs, R, T);
	/* 将旋转向量转换为相对应的旋转矩阵 */
	Rodrigues(R, R);
//	R.convertTo(R, CV_64F);
//	T.convertTo(T, CV_32FC1);

}

