#include "pch.h"
#include "BLCalibration.h"

bool bl::checkImg(cv::Mat img,cv::Size boardSize)
{
	
	if (img.empty()) 
	{
		std::cout << "ͼƬ����\n";
		return false;
	}

   /* �궨����ÿ�С��еĽǵ��� */
	std::vector<cv::Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	if (0 == cv::findChessboardCorners(img, boardSize, image_points_buf)) {
		return false;
	}
	return true;
}