#include "pch.h"
#include "utils.h"
#include "BLCalibration.h"

void bl::hand2eyeCalibration(std::string img_path_file_name, std::string robot_pos_file_name,
	cv::Mat& H_cam2base, const cv::Mat cameraMatrix, const cv::Mat distCoeffs)
{
	vMat R_obj2cam;
	vMat T_obj2cam;
	vMat H_obj2cam;
	vMat R_base2end;
	vMat T_base2end;
	vMat H_base2end;

	cv::Mat R_cam2base;
	cv::Mat T_cam2base;

	// 获取图片RT
	std::ifstream fin(img_path_file_name);
	std::string filename;

	cv::Mat R, T,H;
	while (getline(fin, filename))
	{
		cv::Mat imageInput = cv::imread(filename);
		getImgRT(imageInput, R, T, cameraMatrix, distCoeffs);
		R_obj2cam.push_back(R);
		T_obj2cam.push_back(T);
		R_T2H(R, T, H);
		H_obj2cam.push_back(H);

	}
	fin.close();


	// 获取机器RT
	std::ifstream fin2(robot_pos_file_name);
	std::string filename2;
	int size = 0;
	while (getline(fin2, filename2))
	{
		size++;
	}
	fin2.close();

	cv::Mat_<double> ToolPose = cv::Mat(size, 6, CV_64FC1);

	ToolPose = readRobotPos(robot_pos_file_name,size);
	cv::Mat temp, tempR, tempT;
	for (int j = 0; j < ToolPose.rows; j++)
	{
		//获得的是end2base
		temp = attitudeVectorToMatrix(ToolPose.row(j), false, "xyz");  //注意seq不是空，机械臂末端坐标系与机器人基坐标系之间的为欧拉角
		
		// 求逆得base2end
		temp = temp.inv();
		
		bl::H2R_T(temp, tempR, tempT);
		H_base2end.push_back(temp);
		R_base2end.push_back(tempR);
		T_base2end.push_back(tempT);
	
	}


	// 求手眼
	cv::calibrateHandEye(R_base2end, T_base2end, R_obj2cam, T_obj2cam, R_cam2base, T_cam2base);

	// 拼接成H
	R_T2H(R_cam2base, T_cam2base,H_cam2base);

	cv::Mat H_came2baseTemp = H_cam2base.clone();

	std::cout << "H_cam2base：\n" << H_cam2base << std::endl;

	std::cout << "======================Check==============================\n";

	double x_avg = 0, y_avg = 0, z_avg = 0;
	for (int i = 0; i < H_obj2cam.size(); i++)
	{
		H_base2end[i].convertTo(H_base2end[i], CV_64F);
		cv::Mat pt = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
		std::cout << i << ":\t";
		cv::Mat wuliao = H_base2end[i] * H_came2baseTemp * H_obj2cam[i] * pt;
		cv::Vec3d hwl(wuliao.at<double>(0, 0), wuliao.at<double>(1, 0), wuliao.at<double>(2, 0));
		std::cout << hwl << std::endl;
		x_avg += hwl(0);
		y_avg += hwl(1);
		z_avg += hwl(2);
	}
	x_avg /= H_obj2cam.size();
	y_avg /= H_obj2cam.size();
	z_avg /= H_obj2cam.size();
	std::cout << "x avg:" << x_avg << std::endl;
	std::cout << "y avg:" << y_avg << std::endl;
	std::cout << "z avg:" << z_avg << std::endl;
	double x_max_err = -DBL_MAX, y_max_err = -DBL_MAX, z_max_err = -DBL_MAX;
	double x_avg_err = 0, y_avg_err = 0, z_avg_err = 0;
	for (int i = 0; i < H_obj2cam.size(); i++)
	{
		H_base2end[i].convertTo(H_base2end[i], CV_64F);
		cv::Mat pt = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
		cv::Mat wuliao = H_base2end[i] * H_came2baseTemp * H_obj2cam[i] * pt;
		cv::Vec3d hwl(wuliao.at<double>(0, 0), wuliao.at<double>(1, 0), wuliao.at<double>(2, 0));
		double a1 = abs(hwl(0) - x_avg);
		double a2 = abs(hwl(1) - y_avg);
		double a3 = abs(hwl(2) - z_avg);
		if (a1 > x_max_err)
		{
			x_max_err = a1;
		}
		if (a2 > y_max_err)
		{
			y_max_err = a2;
		}
		if (a3 > z_max_err)
		{
			z_max_err = a3;
		}
		x_avg_err += a1;
		y_avg_err += a2;
		z_avg_err += a3;
	}
	x_avg_err /= H_obj2cam.size();
	y_avg_err /= H_obj2cam.size();
	z_avg_err /= H_obj2cam.size();

	std::cout << "x ave err：  " << x_avg_err << "mm" << "               " << "max err:    " << x_max_err << "mm" << std::endl;
	std::cout << "y avg err：  " << y_avg_err << "mm" << "               " << "max err:    " << y_max_err << "mm" << std::endl;
	std::cout << "z avg err：  " << z_avg_err << "mm" << "               " << "max err:    " << z_max_err << "mm" << std::endl;



}