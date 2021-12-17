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

	// ��ȡͼƬRT
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
		H_base2end.push_back(temp);
		R_base2end.push_back(tempR);
		T_base2end.push_back(tempT);
	
	}


	// ������
	cv::calibrateHandEye(R_base2end, T_base2end, R_obj2cam, T_obj2cam, R_cam2base, T_cam2base);

	// ƴ�ӳ�H
	R_T2H(R_cam2base, T_cam2base,H_cam2base);

	cv::Mat H_came2baseTemp = H_cam2base.clone();
	// ��֤
	//for (int i = 0; i < size; i++)
	//{
	//	std::cout << " " << i << ": " << H_obj2cam[i] * H_came2baseTemp * H_base2end[i]<<std::endl;

	//}



}