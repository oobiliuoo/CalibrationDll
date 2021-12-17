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
	// 验证
	//for (int i = 0; i < size; i++)
	//{
	//	std::cout << " " << i << ": " << H_obj2cam[i] * H_came2baseTemp * H_base2end[i]<<std::endl;

	//}



}