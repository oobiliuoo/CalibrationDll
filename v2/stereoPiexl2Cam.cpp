#include "pch.h"
#include "BLCalibration.h"

cv::Point3f bl::stereoPiexl2Cam(cv::Point2f& piexl_l, cv::Point2f& piexl_r,
	cv::Mat intrinsic_l, cv::Mat intrinsic_r, cv::Mat h_obj2cam_l, cv::Mat h_obj2cam_r)
{

		//  [u1]      |X|					  [u2]      |X|
		//Z*|v1| = Ml*|Y|					Z*|v2| = Mr*|Y|
		//  [ 1]      |Z|					  [ 1]      |Z|
		//			  |1|								|1|


	cv::Mat rt_l = h_obj2cam_l(cv::Rect(0, 0, 4, 3));
	cv::Mat rt_r = h_obj2cam_r(cv::Rect(0, 0, 4, 3));

	if (rt_l.type() != 5)
		rt_l.convertTo(rt_l, CV_32F);
	if (rt_r.type() != 5)
		rt_r.convertTo(rt_r, CV_32F);
	if (intrinsic_l.type() != 5)
		intrinsic_l.convertTo(intrinsic_l, CV_32F);
	if (intrinsic_r.type() != 5)
		intrinsic_r.convertTo(intrinsic_r, CV_32F);

	//rt_l.convertTo(rt_l, CV_32F);
	//rt_r.convertTo(rt_r, CV_32F);
	//intrinsic_l.convertTo(intrinsic_l, CV_32F);
	//intrinsic_r.convertTo(intrinsic_r, CV_32F);
	

//	std::cout << "rt_1\n" << rt_l << std::endl;
//	std::cout << "rt_r\n" << rt_r << std::endl;
//	std::cout << "intrinsic_l\n" << intrinsic_l << std::endl;
//	std::cout << "intrinsic_r\n" << intrinsic_r << std::endl;


	cv::Mat M_l = intrinsic_l * rt_l;
//	std::cout<<"左相机M矩阵 =\n "<<std::endl<< M_l << std::endl;
	cv::Mat M_r = intrinsic_r * rt_r;
//	std::cout<<"右相机M矩阵 =\n "<< std::endl<< M_r << std::endl;

//	std::cout << "1:\n" << piexl_l << std::endl;
//	std::cout << "2:\n" << piexl_r << std::endl;

	//最小二乘法A矩阵
	cv::Mat A = cv::Mat(4, 3, CV_32F);
	A.at<float>(0, 0) = piexl_l.x * M_l.at<float>(2, 0) - M_l.at<float>(0, 0);
	A.at<float>(0, 1) = piexl_l.x * M_l.at<float>(2, 1) - M_l.at<float>(0, 1);
	A.at<float>(0, 2) = piexl_l.x * M_l.at<float>(2, 2) - M_l.at<float>(0, 2);

	A.at<float>(1, 0) = piexl_l.y * M_l.at<float>(2, 0) - M_l.at<float>(1, 0);
	A.at<float>(1, 1) = piexl_l.y * M_l.at<float>(2, 1) - M_l.at<float>(1, 1);
	A.at<float>(1, 2) = piexl_l.y * M_l.at<float>(2, 2) - M_l.at<float>(1, 2);

	A.at<float>(2, 0) = piexl_r.x * M_r.at<float>(2, 0) - M_r.at<float>(0, 0);
	A.at<float>(2, 1) = piexl_r.x * M_r.at<float>(2, 1) - M_r.at<float>(0, 1);
	A.at<float>(2, 2) = piexl_r.x * M_r.at<float>(2, 2) - M_r.at<float>(0, 2);

	A.at<float>(3, 0) = piexl_r.y * M_r.at<float>(2, 0) - M_r.at<float>(1, 0);
	A.at<float>(3, 1) = piexl_r.y * M_r.at<float>(2, 1) - M_r.at<float>(1, 1);
	A.at<float>(3, 2) = piexl_r.y * M_r.at<float>(2, 2) - M_r.at<float>(1, 2);

	//最小二乘法B矩阵
	cv::Mat B = cv::Mat(4, 1, CV_32F);
	B.at<float>(0, 0) = M_l.at<float>(0, 3) - piexl_l.x * M_l.at<float>(2, 3);
	B.at<float>(1, 0) = M_l.at<float>(1, 3) - piexl_l.y * M_l.at<float>(2, 3);
	B.at<float>(2, 0) = M_r.at<float>(0, 3) - piexl_r.x * M_r.at<float>(2, 3);
	B.at<float>(3, 0) = M_r.at<float>(1, 3) - piexl_r.y * M_r.at<float>(2, 3);

//	std::cout << "A:\n" << A << std::endl;
//	std::cout << "B:\n" << B << std::endl;
	cv::Mat XYZ = cv::Mat(3, 1, CV_32F);
	//采用SVD最小二乘法求解XYZ
	solve(A, B, XYZ, cv::DECOMP_SVD);

//	std::cout<<"空间坐标为 = "<< std::endl<<XYZ<< std::endl;

	//世界坐标系中坐标
	cv::Point3f world;
	world.x = XYZ.at<float>(0, 0);
	world.y = XYZ.at<float>(1, 0);
	world.z = XYZ.at<float>(2, 0);

	return world;

}


cv::Point3f bl::stereoPiexl2Cam2(cv::Point2f piexl_l, cv::Point2f piexl_r,
	cv::Mat intrinsic_l, cv::Mat intrinsic_r, cv::Mat h_obj2cam_l, cv::Mat h_obj2cam_r)
{

	//  [u1]      |X|					  [u2]      |X|
	//Z*|v1| = Ml*|Y|					Z*|v2| = Mr*|Y|
	//  [ 1]      |Z|					  [ 1]      |Z|
	//			  |1|								|1|


	std::cout << "1:\n" << piexl_l << std::endl;
	std::cout << "2:\n" << piexl_r << std::endl;


	//世界坐标系中坐标
	cv::Point3f world;

	return world;

}
