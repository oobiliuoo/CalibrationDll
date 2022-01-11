#include "MyTest.h"
#include "BLCalibration.h"
#include "utils.h"

void MyTest::test1()
{

	cv::Mat color = cv::imread("E:\\biliu\\workspace\\Git\\CalibrationDll\\img\\0.PNG");



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


	int index[4] = { 56,6,40,0 };

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




	cv::Mat c = (cv::Mat_<double>(3, 3) << 454.27054, 0, 326.42542, 0, 454.27054, 245.09856, 0, 0, 1);
	cv::Mat k = (cv::Mat_<double>(1, 5) << 0.04648158, -0.0685723, 0.00047238, -0.0001669, 0);



	cv::Mat r1, t1, h1;
	bl::getImgRT(color, r1, t1, c, k);
	bl::R_T2H(r1, t1, h1);
	h1 = h1.clone();
	//	std::cout << "r1\n" << r1 << std::endl;
	//	std::cout << "t1\n" << t1 << std::endl;
	std::cout << "取所有点进行求解：\n" << h1 << std::endl;

	cv::Mat tmp = (cv::Mat_<double>(4, 1) << object_points[index[3]].x, object_points[index[3]].y, object_points[index[3]].z, 1.0);
	cv::Mat tmp2 = h1 * tmp;
	cv::Point3d p3(tmp2.at<double>(0, 0), tmp2.at<double>(1, 0), tmp2.at<double>(2, 0));
	std::cout << "反求的第四点:" << p3 << std::endl;
	cv::Point3d camp;
	bl::piexl2Cam(image_points_buf[index[3]], camp, 185.345, c);
//	std::cout << "camp:" << camp<<std::endl;


	std::cout << "============================\n";
	cv::Mat R, T, H;

	bl::dealP3P(p3d, p2d, c,k, R, T, bl::SOLVEPNP_P3P);
//	std::cout << "r:\n" << R << std::endl;
//	std::cout << "t:\n" << T << std::endl;
	cv::Mat h2;
	bl::R_T2H(R, T, h2);
	std::cout << "取三点进行求解:\n" << h2;
	cv::Mat tmp22 = h2 * tmp;
	cv::Point3d p32(tmp22.at<double>(0, 0), tmp22.at<double>(1, 0), tmp22.at<double>(2, 0));
	std::cout << "\n反求的第四点:" << p32 << std::endl;





	cv::circle(color, image_points_buf[index[0]], 30, cv::Scalar(255, 0, 0));
	cv::circle(color, image_points_buf[index[1]], 30, cv::Scalar(0, 255, 0));
	cv::circle(color, image_points_buf[index[2]], 30, cv::Scalar(0, 0, 255));
	cv::circle(color, image_points_buf[index[3]], 30, cv::Scalar(0, 255, 255));

	cv::imshow("img", color);
	cv::waitKey(0);

}

void MyTest::test2()
{
	char buffRecv[1024];
	cv::Mat_<double> toolPose;
	std::string fileName = "robotPos.txt";
//	std::string pos = "100 200 300 1.0000 2.16546 454.1545";
	int i = 0;
	while (i < 2) {
	
		memset(buffRecv, 0, 1024);
		strcpy_s(buffRecv, "100 200 300 1.0000 2.16546 454.1545");
		toolPose = bl::analyzePose(buffRecv, sizeof(buffRecv), toolPose);
		i++;
	}
	bl::writeRobotPos(toolPose, fileName);

	cv::Mat_<double> t = cv::Mat(i, 6, CV_64FC1);
	t = bl::readRobotPos(fileName,i);
	
	std::cout << "analyzePose_ToolPose  = " << t << std::endl;
	//cv::Mat temp,tempR,tempT;
	//for (int j = 0; j < t.rows; j++)
	//{
	//	temp = attitudeVectorToMatrix(t.row(j), false, "xyz");  //注意seq不是空，机械臂末端坐标系与机器人基坐标系之间的为欧拉角
	//	bl::H2R_T(temp, tempR, tempT);
	//	/*cout << j << "::" << temp << endl;*/
	//	std::cout << j << "::" << tempR << std::endl;
	//	std::cout << j << "::" << tempT << std::endl;
	//}


	std::string img_file = "img.txt";
	
	cv::Mat c = (cv::Mat_<double>(3, 3) << 454.27054, 0, 326.42542, 0, 454.27054, 245.09856, 0, 0, 1);
	cv::Mat k = (cv::Mat_<double>(1, 5) << 0.04648158, -0.0685723, 0.00047238, -0.0001669, 0);

	cv::Mat h;
	bl::hand2eyeCalibration(img_file,fileName,h,c,k);
	std::cout << "over";

}

void MyTest::test3()
{

	cv::Mat c = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 摄像机内参数矩阵 */
	cv::Mat k = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	std::string file = "eye2HandCalImageName.txt";

	bl::cameraCalibration(file,c,k);

	cv::Mat img = cv::imread("E:/biliu/workspace/Git/Eye2Hand/img/eyeHandCal/0.PNG");
	cv::Size board_size = cv::Size(11, 8);    /* 标定板上每行、列的角点数 */
	std::cout << "check:" << bl::checkImg(img, board_size);

}

void MyTest::test4()
{

	double a[9] = {-32,730,517,-174.984,-5.217,86,540};
	cv::Mat j6pos = cv::Mat(1, 6, CV_64F, a);

	cv::Point3d p(-5.367, 781.840, 241.916);
	cv::Mat t;
	bl::Tool2J6(j6pos,p,t);

}
