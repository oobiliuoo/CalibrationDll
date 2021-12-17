#include "pch.h"
#include "BLCalibration.h"

bool bl::checkImg(cv::Mat img,cv::Size boardSize)
{
	
	if (img.empty()) 
	{
		std::cout << "图片错误\n";
		return false;
	}

   /* 标定板上每行、列的角点数 */
	std::vector<cv::Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
	if (0 == cv::findChessboardCorners(img, boardSize, image_points_buf)) {
		return false;
	}
	return true;
}