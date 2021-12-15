#include "pch.h"
#include "utils.h"
#include "BLCalibration.h"

void bl::hand2eyeCalibration(std::string img_path_file_name, std::string robot_pos_file_name,
	cv::Mat& H_cam2base, const cv::Mat cameraMatrix, const cv::Mat distCoeffs)
{
	vMat R_obj2cam;
	vMat T_obj2cam;
	vMat R_base2end;
	vMat T_base2end;

	cv::Mat R_cam2base;
	cv::Mat T_cam2base;

	// ��ȡͼƬRT
	std::ifstream fin(img_path_file_name);
	std::string filename;
	while (getline(fin, filename))
	{
		cv::Mat R, T;
		cv::Mat imageInput = cv::imread(filename);
		getImgRT(imageInput, R, T, cameraMatrix, distCoeffs);
		R_obj2cam.push_back(R);
		T_obj2cam.push_back(T);

	}
	fin.close();


	// ��ȡ����RT
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
		//��õ���end2base
		temp = attitudeVectorToMatrix(ToolPose.row(j), false, "xyz");  //ע��seq���ǿգ���е��ĩ������ϵ������˻�����ϵ֮���Ϊŷ����
		
		// �����base2end
		temp = temp.inv();

		bl::H2R_T(temp, tempR, tempT);
		R_base2end.push_back(tempR);
		T_base2end.push_back(tempT);
	
	}


	// ������
	cv::calibrateHandEye(R_base2end, T_base2end, R_obj2cam, T_obj2cam, R_cam2base, T_cam2base);

	// ƴ�ӳ�H
	R_T2H(R_cam2base, T_cam2base,H_cam2base);



}