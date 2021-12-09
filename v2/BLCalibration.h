#pragma once
namespace bl
{


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
	_declspec(dllexport) void piexl2Cam(cv::Point2d piexl, cv::Point3f& camPoint, double zc, cv::Mat cameraMatrix);


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



	_declspec(dllexport) int dealP3P(std::vector<cv::Point3f> Points3D,std::vector<cv::Point2f> Points2D,
		const cv::Mat cameraMatrix, const cv::Mat distCoeffs,cv::Mat& R, cv::Mat& T,int method);
	


}

