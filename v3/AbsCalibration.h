#pragma once
#include "pch.h"

namespace bl {
	

	class AbsCalibration {
	public:
		/*
		标定类型
		CamCalibration: 1
		ToolCalibration: 2
		*/
		virtual int mType() = 0;

		virtual ~AbsCalibration() {};

	};


	class CLASS_DECLSPEC AbsCamCalibration : public AbsCalibration
	{
	public:
		AbsCamCalibration(){};

		~AbsCamCalibration(){};

		/*
		相机标定
		输入：
			img_path_file_name: 照片路径文件名
		输出：
			cameraMatrix:		相机内参
			distCoeffs：		畸变系数
		注：根据图像路径进行标定
		*/
		virtual int monocularCalbration(std::string img_path_file_name, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
			cv::Size board_size = cv::Size(11, 8), cv::Size square_size = cv::Size(5, 5)) = 0;

		// 双目相机标定
		//	void stereoCamCalbration() {};


	};


	class CLASS_DECLSPEC AbsToolCalibration : public AbsCalibration
	{

	protected:
		/*左相机内参*/
		cv::Mat intrinsic_L;
		/*右相机内参*/
		cv::Mat intrinsic_R;
		/*左相机到右相机的转换矩阵*/
		cv::Mat H_left2right;
		/*左相机的手眼矩阵*/
		cv::Mat H_cam2obj_L;
		/*右相机的手眼矩阵*/
		cv::Mat H_cam2obj_R;
		/*工具端矩阵*/
		cv::Mat H_tool2end;
		/*左右阈值*/
		int threshold_data[2] = {0,0};
		/*左图区域*/
		cv::Rect rect_L;
		/*右图区域*/
		cv::Rect rect_R;
		/*显示设置*/
		int cout_f = 0;

	public:
		AbsToolCalibration() {};

		~AbsToolCalibration() {};

		/*
			设置左相机内参
			输入： 
				src: 目标值（3*3）
		*/
		virtual void setIntrinsic_l(cv::Mat src) = 0;

		/*
			设置右相机内参
			输入：
				src: 目标值（3*3）
		*/
		virtual	void setIntrinsic_r(cv::Mat src) = 0;

		/*
			设置左相机到右相机的转换矩阵
			输入：
				src: 目标值（3*3）
		*/
		virtual void setH_left2right(cv::Mat src) = 0;

		/*
			设置左相机到右相机的转换矩阵
				输入：
					src: 目标值（3*3）
		*/
		virtual void setH_cam2obj_l(cv::Mat src) = 0;

		/*
			设置左图区域
			输入：
					rect: 目标区域
		*/
		virtual void setRect_L(cv::Rect rect) = 0;

		/*
			设置右图区域
			输入：
					rect: 目标区域
		*/
		virtual void setRect_R(cv::Rect rect) = 0;

		/*
			设置左右阈值
			输入：
				agr1: 左图阈值
				agr2: 右图阈值
		*/
		virtual void setThreshold(int, int) = 0;

		/*
			设置显示模式
			输入：
				arg: 0 无 1 点输出 2 图输出 3 找点调试 4 计算数据输出
		*/
		virtual void setCout(int) = 0;

		/*
			获取工具端平移向量
			return : 工具端平移向量
		*/
		virtual cv::Mat getT_tool2end() = 0;

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
		virtual int stereoToolRepair(cv::Mat imgL, cv::Mat imgR, cv::Mat endPos, cv::Mat H_cam2base_L,
			cv::Mat intrinsic_L, cv::Mat intrinsic_R, cv::Mat H_left2right,
			cv::Mat& T_tool2end, int cout_flog = 0, cv::Rect rect = cv::Rect(0, 0, 0, 0)) = 0;

		/*
			双目工具端修正
			输入：
				imgL:左图
				imgR:右图
				endPos: 末端位姿 （2*3）<<x,y,z,rx,ry,rz;
			return: 0 正常结束
			注： 需设置好参数
		*/
		virtual	int stereoToolRepair(cv::Mat imgL, cv::Mat imgR, cv::Mat endPos) = 0;

		/*
			双目工具端修正
			输入：
				point_L:左图点
				point_R:右图点
				endPos: 末端位姿 （2*3）<<x,y,z,rx,ry,rz;
			return: 0 正常结束
			注： 需设置好参数
		*/
		virtual	int stereoToolRepair(cv::Point2f point_L, cv::Point2f point_R, cv::Mat endPos) = 0;

		/*
			找点
			输入：
				src: 目标图
				show: 输出标志
		*/
		virtual cv::Point2d Cusp(cv::Mat src, int show) = 0;

		/*
			找点
			输入：
				src: 目标图
				arg: 阈值
		*/
		virtual cv::Point2d Cusp2(cv::Mat src,int arg) = 0;

		/*
			双目像素点转世界坐标
			输入：
				piexl_l:左图点
				piexl_r:右图点
			注： 需先设置好参数
				
		*/
		virtual cv::Point3f stereoPiexl2Obj(cv::Point2f& piexl_l, cv::Point2f& piexl_r) = 0;

		/*
			求工具端转换矩阵
			输入：
				j6pos: 法兰盘位姿
			toolPoint: 工具端基底坐标
		*/
		virtual void Tool2End(cv::Mat j6Pos, cv::Point3d toolPoint) = 0;

	};

}
