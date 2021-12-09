#include "MyTest.h"

#include "BLCalibration.h"

void MyTest::test1()
{
	
	cv::Mat color = cv::imread("E:\\biliu\\workspace\\CppCode\\test\\AstraTest\\AstraCamera\\img\\image_7.jpg");
	
	std::vector<cv::Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	cv::Size board_size = cv::Size(11, 8);    /* �궨����ÿ�С��еĽǵ��� */
	/*������ά��Ϣ*/
	cv::Size square_size = cv::Size(5, 5);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
		/* ��ȡ�ǵ� */
	if (0 == findChessboardCorners(color, board_size, image_points_buf))
	{
		std::cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�
		return;
	}
	else
	{
		cv::Mat view_gray;
		cvtColor(color, view_gray, CV_RGB2GRAY);
		/* �����ؾ�ȷ�� */
		find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(11, 11)); //�Դ���ȡ�Ľǵ���о�ȷ��

	}
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


	std::vector<cv::Point3f> p3d;
	std::vector<cv::Point2f> p2d;

	p3d.push_back(object_points[0]);
	p3d.push_back(object_points[4]);
	p3d.push_back(object_points[33]);
	p3d.push_back(object_points[37]);


	p2d.push_back(image_points_buf[0]);
	p2d.push_back(image_points_buf[4]);
	p2d.push_back(image_points_buf[33]);
	p2d.push_back(image_points_buf[37]);

	cv::Mat c = (cv::Mat_<float>(3,3)<<454.27054,0,326.42542,0,454.27054,245.09856,0,0,1);
	cv::Mat k = (cv::Mat_<float>(1,5)<<0.04648158,-0.0685723,0.00047238,-0.0001669,0);



	cv::Mat r1, t1, h1;
	bl::getImgRT(color, r1, t1, c, k);
	bl::R_T2H(r1, t1, h1);
	//	std::cout << "r1\n" << r1 << std::endl;
	//	std::cout << "t1\n" << t1 << std::endl;
	std::cout << "h\n" << h1 << std::endl;
	
	


	cv::Mat R, T, H;

	bl::dealP3P(p3d, p2d, c,k, R, T, bl::SOLVEPNP_BL);


}
