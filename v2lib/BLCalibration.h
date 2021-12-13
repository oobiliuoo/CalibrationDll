#pragma once
namespace bl
{


	using vP3F = std::vector<cv::Point3f>;
	using vP3D = std::vector<cv::Point3d>;
	using vP2F = std::vector<cv::Point2f>;


	enum SolvePNPMethod {
		 SOLVEPNP_ITERATIVE   = 0,
		 SOLVEPNP_EPNP        = 1, 
		 SOLVEPNP_P3P         = 2, 
		 SOLVEPNP_BL		  = 100
	};



	/*
		将R,T矩阵拼接成H矩阵
		输入：
			R：（3*3）旋转矩阵
			T：（3*1）平移向量
		输出：
			H：（4*4）齐次矩阵
	*/
	_declspec(dllexport) void R_T2H(cv::Mat R,cv::Mat T,cv::Mat& H);

	/*
		将H矩阵拆分成R,T矩阵
		输入：
			H：（4*4）齐次矩阵
		输出：
			R：（3*3）旋转矩阵
			T：（3*1）平移向量
	*/
	_declspec(dllexport) void H2R_T(cv::Mat H,cv::Mat& R,cv::Mat& T);

	/*
		相机标定
		输入：
		img_path_file_name: 照片路径文件名
		输出：
		cameraMatrix:		相机内参
		distCoeffs：			畸变系数
		注：					根据图像路径进行标定
	*/
	_declspec(dllexport) void cameraCalibration(std::string img_path_file_name, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

	/*
		像素坐标转相机坐标
		输入：
		piexl:		    像素坐标
		zc:				像素点的深度信息
		cameraMatrix:	相机内参
		输出：
		camPoint:		相机坐标
	*/
	_declspec(dllexport) void piexl2Cam(cv::Point2d piexl, cv::Point3f& camPoint, double zc, cv::Mat cameraMatrix);


	/*
		获取图像到相机的R、T
		输入：
			srcImg:				指定图像
			cameraMatrix:		相机内参
			distCoeffs：			畸变系数
		输出：
			R：（3*3）旋转矩阵
			T：（3*1）平移向量
	*/
	_declspec(dllexport) void getImgRT(const cv::Mat srcImg, cv::Mat& R, cv::Mat& T, const cv::Mat cameraMatrix, const cv::Mat distCoeffs);



	/*
		求解P3P问题
		输入：
			Points3D:			三维点集
			points2d:			二维点集
			cameraMatrix:		相机内参
			distCoeffs：			畸变系数
			method:				求解方法
		输出：
			R：（3*3）旋转矩阵
			T：（3*1）平移向量
		注： method为SOLVEPNP_ITERATIVE或SOLVEPNP_P3P时只需4个点   
	*/
	_declspec(dllexport) int dealP3P(std::vector<cv::Point3d> Points3D,std::vector<cv::Point2f> Points2D,
		const cv::Mat cameraMatrix, const cv::Mat distCoeffs,cv::Mat& R, cv::Mat& T,int method);
	

	/*
		求解两点之间的距离
		o:点1
		p:点2
		return: 距离
	*/
	_declspec(dllexport) double get2PointD(cv::Point3f o, cv::Point3f p);


	/*
		求解余弦值
		cos<a,b> = (a*a+b*b-c*c)/(2ab)
	*/
	_declspec(dllexport) double getCosines(double a, double b, double c);

	
	_declspec(dllexport) void ICP(vP3D originPoints,vP3D targetPoints,cv::Mat& R,cv::Mat& T);


}

