#pragma once
#include "pch.h"

namespace bl {


	class AbsUtils {

	public:
		virtual ~AbsUtils(){};
	};

	class CLASS_DECLSPEC AbsTransUtils : public AbsUtils
	{
	public:
		/*
		将H矩阵拆分成R,T矩阵
		输入：
			H：（4*4）齐次矩阵
		输出：
			R：（3*3）旋转矩阵
			T：（3*1）平移向量
		*/
		virtual void H2R_T(cv::Mat H, cv::Mat& R, cv::Mat& T) = 0;

		/*
		将R,T矩阵拼接成H矩阵
		输入：
		R：（3*3）旋转矩阵
		T：（3*1）平移向量
		输出：
		H：（4*4）齐次矩阵
		*/
		virtual void R_T2H(cv::Mat R, cv::Mat T, cv::Mat& H) = 0;


		virtual void to_CV_32F(cv::Mat& src) = 0;


		/**************************************************
		* @brief   欧拉角转换为旋转矩阵
		* @note
		* @param    const std::string& seq  指定欧拉角的排列顺序；（机械臂的位姿类型有xyz,zyx,zyz几种，需要区分）
		* @param    const Mat& eulerAngle   欧拉角（1*3矩阵）, 角度值
		* @param
		* @return   返回3*3旋转矩阵
		**************************************************/
		virtual cv::Mat eulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const std::string& seq) = 0;

		/**************************************************
		* @brief      将采集的原始数据转换为齐次矩阵（从机器人控制器中获得的）
		* @note
		* @param	  Mat& m    1*6//1*10矩阵 ， 元素为： x,y,z,rx,ry,rz  or x,y,z, q0,q1,q2,q3,rx,ry,rz
		* @param	  bool useQuaternion      原始数据是否使用四元数表示
		* @param	  string& seq         原始数据使用欧拉角表示时，坐标系的旋转顺序
		* @return	  返回转换完的齐次矩阵
		**************************************************/
		virtual cv::Mat attitudeVectorToMatrix(const cv::Mat& m, bool useQuaternion, const std::string& seq) = 0;

		/**************************************************
		* @brief	检查是否是旋转矩阵
		* @note
		* @param
		* @param
		* @param
		* @return  true : 是旋转矩阵， false : 不是旋转矩阵
		**************************************************/
		virtual bool isRotatedMatrix(cv::Mat& R) = 0;

		/**************************************************
		* @brief   将四元数转换为旋转矩阵
		* @note
		* @param   const Vec4d& q   归一化的四元数: q = q0 + q1 * i + q2 * j + q3 * k;
		* @return  返回3*3旋转矩阵R
		**************************************************/
		virtual cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q) = 0;

	};
}


