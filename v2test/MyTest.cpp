#include "MyTest.h"

#include "BLCalibration.h"

void MyTest::test1()
{
	
	cv::Mat color = cv::imread("E:\\biliu\\workspace\\CppCode\\test\\AstraTest\\AstraCamera\\img\\image_12.jpg");

	
	std::vector<cv::Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
	cv::Size board_size = cv::Size(11, 8);    /* 标定板上每行、列的角点数 */
	/*棋盘三维信息*/
	cv::Size square_size = cv::Size(5, 5);  /* 实际测量得到的标定板上每个棋盘格的大小 */
		/* 提取角点 */
	if (0 == findChessboardCorners(color, board_size, image_points_buf))
	{
		std::cout << "can not find chessboard corners!\n"; //找不到角点
		return;
	}
	else
	{
		cv::Mat view_gray;
		cvtColor(color, view_gray, CV_RGB2GRAY);
		/* 亚像素精确化 */
		find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(11, 11)); //对粗提取的角点进行精确化

	}
	/* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	std::vector<cv::Point3d> object_points;
	for (i = 0; i < board_size.height; i++)
	{
		for (j = 0; j < board_size.width; j++)
		{
			cv::Point3d realPoint;
			/* 假设标定板放在世界坐标系中z=0的平面上 */
			realPoint.x = i * square_size.width;
			realPoint.y = j * square_size.height;
			realPoint.z = 0;
			object_points.push_back(realPoint);
		}
	}


	std::vector<cv::Point3d> p3d;
	std::vector<cv::Point2f> p2d;


	int index[4] = { 0,4,35,37 };

//	object_points[index[0]].z = 5;
//	object_points[index[1]].z = 15;
//	object_points[index[2]].z = 25;
//	object_points[index[3]].z = 0;

	p3d.push_back(object_points[index[0]]);
	p3d.push_back(object_points[index[1]]);
	p3d.push_back(object_points[index[2]]);
	p3d.push_back(object_points[index[3]]);


	p2d.push_back(image_points_buf[index[0]]);
	p2d.push_back(image_points_buf[index[1]]);
	p2d.push_back(image_points_buf[index[2]]);
	p2d.push_back(image_points_buf[index[3]]);




	cv::Mat c = (cv::Mat_<double>(3,3)<<454.27054,0,326.42542,0,454.27054,245.09856,0,0,1);
	cv::Mat k = (cv::Mat_<double>(1,5)<<0.04648158,-0.0685723,0.00047238,-0.0001669,0);



	cv::Mat r1, t1, h1;
	bl::getImgRT(color, r1, t1, c, k);
	bl::R_T2H(r1, t1, h1);
	//	std::cout << "r1\n" << r1 << std::endl;
	//	std::cout << "t1\n" << t1 << std::endl;
	std::cout << "h\n" << h1 << std::endl;
	
	


	cv::Mat R, T, H;

	bl::dealP3P(p3d, p2d, c,k, R, T, bl::SOLVEPNP_P3P);
//	std::cout << "r:\n" << R << std::endl;
//	std::cout << "t:\n" << T << std::endl;
	cv::Mat h2;
	bl::R_T2H(R, T, h2);
	std::cout << "h2:" << h2;



	cv::circle(color, image_points_buf[index[0]], 30, cv::Scalar(0, 0, 255));
	cv::circle(color, image_points_buf[index[1]], 30, cv::Scalar(0, 0, 255));
	cv::circle(color, image_points_buf[index[2]], 30, cv::Scalar(0, 0, 255));
	cv::circle(color, image_points_buf[index[3]], 30, cv::Scalar(0, 255, 255));

	cv::imshow("img", color);
	cv::waitKey(0);

}
