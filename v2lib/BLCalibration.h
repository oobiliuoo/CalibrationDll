#pragma once
namespace bl
{


	using vP3F = std::vector<cv::Point3f>;
	using vP3D = std::vector<cv::Point3d>;
	using vP2F = std::vector<cv::Point2f>;
	using vMat = std::vector<cv::Mat>;


	enum SolvePNPMethod {
		 SOLVEPNP_ITERATIVE   = 0,
		 SOLVEPNP_EPNP        = 1, 
		 SOLVEPNP_P3P         = 2, 
		 SOLVEPNP_BL		  = 100
	};



	/*
		��R,T����ƴ�ӳ�H����
		���룺
			R����3*3����ת����
			T����3*1��ƽ������
		�����
			H����4*4����ξ���
	*/
	_declspec(dllexport) void R_T2H(cv::Mat R,cv::Mat T,cv::Mat& H);

	/*
		��H�����ֳ�R,T����
		���룺
			H����4*4����ξ���
		�����
			R����3*3����ת����
			T����3*1��ƽ������
	*/
	_declspec(dllexport) void H2R_T(cv::Mat H,cv::Mat& R,cv::Mat& T);

	/*
		����궨
		���룺
		img_path_file_name: ��Ƭ·���ļ���
		�����
		cameraMatrix:		����ڲ�
		distCoeffs��			����ϵ��
		ע��					����ͼ��·�����б궨
	*/
	_declspec(dllexport) void cameraCalibration(std::string img_path_file_name, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

	/*
		��������ת�������
		���룺
		piexl:		    ��������
		zc:				���ص�������Ϣ
		cameraMatrix:	����ڲ�
		�����
		camPoint:		�������
	*/
	_declspec(dllexport) void piexl2Cam(cv::Point2d piexl, cv::Point3d& camPoint, double zc, cv::Mat cameraMatrix);


	/*
		��ȡͼ�������R��T
		���룺
			srcImg:				ָ��ͼ��
			cameraMatrix:		����ڲ�
			distCoeffs��			����ϵ��
		�����
			R����3*3����ת����
			T����3*1��ƽ������
	*/
	_declspec(dllexport) void getImgRT(const cv::Mat srcImg, cv::Mat& R, cv::Mat& T, const cv::Mat cameraMatrix, const cv::Mat distCoeffs);



	/*
		���P3P����
		���룺
			Points3D:			��ά�㼯
			points2d:			��ά�㼯
			cameraMatrix:		����ڲ�
			distCoeffs��			����ϵ��
			method:				��ⷽ��
		�����
			R����3*3����ת����
			T����3*1��ƽ������
		ע�� methodΪSOLVEPNP_ITERATIVE��SOLVEPNP_P3Pʱֻ��4����   
	*/
	_declspec(dllexport) int dealP3P(std::vector<cv::Point3d> Points3D,std::vector<cv::Point2f> Points2D,
		const cv::Mat cameraMatrix, const cv::Mat distCoeffs,cv::Mat& R, cv::Mat& T,int method);
	

	/*
		�������֮��ľ���
		o:��1
		p:��2
		return: ����
	*/
	_declspec(dllexport) double get2PointD(cv::Point3f o, cv::Point3f p);


	/*
		�������ֵ
		cos<a,b> = (a*a+b*b-c*c)/(2ab)
	*/
	_declspec(dllexport) double getCosines(double a, double b, double c);


	/*
		ICP������������֮���R��T
		���룺
			originPoints:Դ����
			targetPoints:Ŀ����
		�����
			R��origin��target����ת����
			T��origin��target��ƽ������
	*/
	_declspec(dllexport) void ICP(vP3D originPoints,vP3D targetPoints,cv::Mat& R,cv::Mat& T);


	/*
		���۱궨���������⣩
		���룺
			img_path_file_name: �궨����Ƭ·���ļ���
			robot_pos_file_name: ����λ���ļ� ��ʽ����x y z w v u��
			cameraMatrix:		����ڲ�
			distCoeffs��			����ϵ��
		�����
			H_cam2base:			���۾���
	*/
	_declspec(dllexport) void hand2eyeCalibration(std::string img_path_file_name, std::string robot_pos_file_name,
		cv::Mat& H_cam2base, const cv::Mat cameraMatrix, const cv::Mat distCoeffs);

	/*
		��������λ��,��ƴ��
		���룺
			bufRecv:   �յ��Ļ���λ��
			size:	   bufRecv����
			ToolPose�� ������λ��
		return:
			λ������
	*/
	_declspec(dllexport) cv::Mat_<double> analyzePose(char* bufRecv, int size, cv::Mat_<double> ToolPose);
	
	/*
		д�������λ��
		���룺
			pos:			������λ������
			robot_pos_name:	���������λ�˵��ļ�
	*/
	_declspec(dllexport) void writeRobotPos(cv::Mat_<double> pos,std::string robot_pos_name);


	/*
		��ȡ������λ��
		���룺
			robot_pos_name:	���������λ�˵��ļ�
			size:			λ�˵ĸ���
		return:
			������λ������
	*/
	_declspec(dllexport) cv::Mat_<double> readRobotPos(std::string robot_pos_name, int size);



	_declspec(dllexport) bool checkImg(cv::Mat img, cv::Size boardSize);

}

