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

	struct BinocularStereoCamera
	{
		cv::Mat _left_cameraMatrix;
		cv::Mat _left_distCoeffs;

		cv::Mat _right_cameraMatrix;
		cv::Mat _right_distCoeffs;

		cv::Mat _H_l2r;

		cv::Mat _E;
		cv::Mat _F;

		cv::Mat _R1;
		cv::Mat _R2;
		cv::Mat _P1;
		cv::Mat _P2;
		cv::Mat _Q;

	};

	using BSC = BinocularStereoCamera;



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
			robot_pos_file_name: ����λ���ļ� ��ʽ����x y z rx ry rz��
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


	/*
		�ж�ͼƬ�Ƿ��ܼ�⵽�ǵ�
		���룺
			img: ͼƬ
			boardSize: �궨���С
		return:
			
			
	*/
	_declspec(dllexport) bool checkImg(cv::Mat img, cv::Size boardSize);

	/*
		���߶�����
		���룺
			j6Pos: ������T0λ��
			toolPoint: ��˻�������
		�����
			H�����߶�ת������
	*/
	_declspec(dllexport) void Tool2J6(cv::Mat  j6Pos, cv::Point3d toolPoint, cv::Mat& H);


	/*
		˫Ŀ�궨
	*/
	_declspec(dllexport) void stereoCalibrate(std::string img_1_path_file_name,std::string img_2_path_file_name,
	const cv::Size& board_size, const cv::Size& square_size,
		bl::BSC& bsc);

	//_declspec(dllexport) void stereoCalibrate(cv::Mat ,cv::Mat,)

	/*
		˫Ŀ����
	*/
	_declspec(dllexport) void stereoCorrect(BSC& bsc,cv::Mat& imgl,cv::Mat& imgr,cv::Rect* validRoi);

	_declspec(dllexport) cv::Point3f stereoPiexl2Cam(cv::Point2f& piexl_l, cv::Point2f& piexl_r,
		cv::Mat intrinsic_l,cv::Mat Intrinsic_r,cv::Mat h_obj2cam_l,cv::Mat h_obj2cam_r);

	_declspec(dllexport) cv::Point3f stereoPiexl2Cam2(cv::Point2f piexl_l, cv::Point2f piexl_r,
		cv::Mat intrinsic_l, cv::Mat Intrinsic_r, cv::Mat h_obj2cam_l, cv::Mat h_obj2cam_r);



	/*
		˫Ŀ���߶�����
		���룺
		imgL:��ͼ
		imgR:��ͼ
		endPos: ĩ��λ�� ��2*3��<<x,y,z,rx,ry,rz;
		H_cam2base_L: ����������۾���
		intrinsic_L: ������ڲ�
		intrinsic_R: ������ڲ�
		H_left2right: ��������������ת������
		cout_flog: �ڲ���� Ĭ�Ϲر� 10086 ��ʾ�ҵ�ͼ 
		rect: ��׼��Χ

		�����
		T_tool2end: ���߶˵�ĩ�˵�ƽ������ ��1*3��<<x,y,z

		return: 0 ��������

	*/
	_declspec(dllexport) int stereoToolRepair(cv::Mat imgL, cv::Mat imgR, cv::Mat endPos, cv::Mat H_cam2base_L,
		cv::Mat intrinsic_L, cv::Mat intrinsic_R, cv::Mat H_left2right,
		cv::Mat& T_tool2end, int cout_flog = 0, cv::Rect rect = cv::Rect(0, 0, 0, 0));

}

