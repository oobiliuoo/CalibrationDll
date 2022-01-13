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
	_declspec(dllexport) void piexl2Cam(cv::Point2d piexl, cv::Point3d& camPoint, double zc, cv::Mat cameraMatrix);


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


	/*
		ICP，求两个点云之间的R，T
		输入：
			originPoints:源点云
			targetPoints:目标云
		输出：
			R：origin到target的旋转矩阵
			T：origin到target的平移向量
	*/
	_declspec(dllexport) void ICP(vP3D originPoints,vP3D targetPoints,cv::Mat& R,cv::Mat& T);


	/*
		手眼标定（眼在手外）
		输入：
			img_path_file_name: 标定板照片路径文件名
			robot_pos_file_name: 机器位姿文件 格式：（x y z rx ry rz）
			cameraMatrix:		相机内参
			distCoeffs：			畸变系数
		输出：
			H_cam2base:			手眼矩阵
	*/
	_declspec(dllexport) void hand2eyeCalibration(std::string img_path_file_name, std::string robot_pos_file_name,
		cv::Mat& H_cam2base, const cv::Mat cameraMatrix, const cv::Mat distCoeffs);

	/*
		解析机器位姿,并拼接
		输入：
			bufRecv:   收到的机器位姿
			size:	   bufRecv长度
			ToolPose： 机器人位姿
		return:
			位姿数组
	*/
	_declspec(dllexport) cv::Mat_<double> analyzePose(char* bufRecv, int size, cv::Mat_<double> ToolPose);
	
	/*
		写入机器人位姿
		输入：
			pos:			机器人位姿数组
			robot_pos_name:	保存机器人位姿的文件
	*/
	_declspec(dllexport) void writeRobotPos(cv::Mat_<double> pos,std::string robot_pos_name);


	/*
		读取机器人位姿
		输入：
			robot_pos_name:	保存机器人位姿的文件
			size:			位姿的个数
		return:
			机器人位姿数组
	*/
	_declspec(dllexport) cv::Mat_<double> readRobotPos(std::string robot_pos_name, int size);


	/*
		判断图片是否能检测到角点
		输入：
			img: 图片
			boardSize: 标定板大小
		return:
			
			
	*/
	_declspec(dllexport) bool checkImg(cv::Mat img, cv::Size boardSize);

	/*
		工具端修正
		输入：
			j6Pos: 机器人T0位姿
			toolPoint: 尖端机底坐标
		输出：
			H：工具端转换矩阵
	*/
	_declspec(dllexport) void Tool2J6(cv::Mat  j6Pos, cv::Point3d toolPoint, cv::Mat& H);


	/*
		双目标定
	*/
	_declspec(dllexport) void stereoCalibrate(std::string img_1_path_file_name,std::string img_2_path_file_name,
	const cv::Size& board_size, const cv::Size& square_size,
		bl::BSC& bsc);

	//_declspec(dllexport) void stereoCalibrate(cv::Mat ,cv::Mat,)

	/*
		双目矫正
	*/
	_declspec(dllexport) void stereoCorrect(BSC& bsc,cv::Mat& imgl,cv::Mat& imgr,cv::Rect* validRoi);

	_declspec(dllexport) cv::Point3f stereoPiexl2Cam(cv::Point2f& piexl_l, cv::Point2f& piexl_r,
		cv::Mat intrinsic_l,cv::Mat Intrinsic_r,cv::Mat h_obj2cam_l,cv::Mat h_obj2cam_r);

	_declspec(dllexport) cv::Point3f stereoPiexl2Cam2(cv::Point2f piexl_l, cv::Point2f piexl_r,
		cv::Mat intrinsic_l, cv::Mat Intrinsic_r, cv::Mat h_obj2cam_l, cv::Mat h_obj2cam_r);



	/*
		双目工具端修正
		输入：
		imgL:左图
		imgR:右图
		endPos: 末端位姿 （2*3）<<x,y,z,rx,ry,rz;
		H_cam2base_L: 左相机的手眼矩阵
		intrinsic_L: 左相机内参
		intrinsic_R: 右相机内参
		H_left2right: 左相机到右相机的转换矩阵
		cout_flog: 内部输出 默认关闭 10086 显示找点图 
		rect: 精准范围

		输出：
		T_tool2end: 工具端到末端的平移向量 （1*3）<<x,y,z

		return: 0 正常结束

	*/
	_declspec(dllexport) int stereoToolRepair(cv::Mat imgL, cv::Mat imgR, cv::Mat endPos, cv::Mat H_cam2base_L,
		cv::Mat intrinsic_L, cv::Mat intrinsic_R, cv::Mat H_left2right,
		cv::Mat& T_tool2end, int cout_flog = 0, cv::Rect rect = cv::Rect(0, 0, 0, 0));

}

