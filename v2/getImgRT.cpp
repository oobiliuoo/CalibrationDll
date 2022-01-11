#include "pch.h"
#include "BLCalibration.h"

void bl::getImgRT(const cv::Mat srcImg, cv::Mat& R, cv::Mat& T, const cv::Mat cameraMatrix, const cv::Mat distCoeffs)
{
	std::vector<cv::Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	cv::Size board_size = cv::Size(11, 8);    /* �궨����ÿ�С��еĽǵ��� */
	/*������ά��Ϣ*/
	cv::Size square_size = cv::Size(5, 5);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
		/* ��ȡ�ǵ� */
	if (0 == findChessboardCornersSB(srcImg, board_size, image_points_buf,  cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY))
	{
		std::cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�
		return ;
	}
	//else
	//{
	//	cv::Mat view_gray;
	//	cvtColor(srcImg, view_gray, CV_RGB2GRAY);
	//	/* �����ؾ�ȷ�� */
	//	find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(11, 11)); //�Դ���ȡ�Ľǵ���о�ȷ��
	//
	//}
	/* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	std::vector<cv::Point3f> object_points;
	for (i = 0; i < board_size.height; i++)
	{
		for (j = 0; j < board_size.width; j++)
		{
			cv::Point3f realPoint;
			/* ����궨�������������ϵ��z=0��ƽ���� */
			realPoint.x = i * square_size.width;
			realPoint.y = j * square_size.height;
			realPoint.z = 0;
			object_points.push_back(realPoint);
		}
	}

	// ��ȡrt����
	cv::solvePnPRansac(object_points, image_points_buf, cameraMatrix, distCoeffs, R, T);
	/* ����ת����ת��Ϊ���Ӧ����ת���� */
	Rodrigues(R, R);
//	R.convertTo(R, CV_64F);
//	T.convertTo(T, CV_32FC1);

}

