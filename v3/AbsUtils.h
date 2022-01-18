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
		��H�����ֳ�R,T����
		���룺
			H����4*4����ξ���
		�����
			R����3*3����ת����
			T����3*1��ƽ������
		*/
		virtual void H2R_T(cv::Mat H, cv::Mat& R, cv::Mat& T) = 0;

		/*
		��R,T����ƴ�ӳ�H����
		���룺
		R����3*3����ת����
		T����3*1��ƽ������
		�����
		H����4*4����ξ���
		*/
		virtual void R_T2H(cv::Mat R, cv::Mat T, cv::Mat& H) = 0;


		virtual void to_CV_32F(cv::Mat& src) = 0;


		/**************************************************
		* @brief   ŷ����ת��Ϊ��ת����
		* @note
		* @param    const std::string& seq  ָ��ŷ���ǵ�����˳�򣻣���е�۵�λ��������xyz,zyx,zyz���֣���Ҫ���֣�
		* @param    const Mat& eulerAngle   ŷ���ǣ�1*3����, �Ƕ�ֵ
		* @param
		* @return   ����3*3��ת����
		**************************************************/
		virtual cv::Mat eulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const std::string& seq) = 0;

		/**************************************************
		* @brief      ���ɼ���ԭʼ����ת��Ϊ��ξ��󣨴ӻ����˿������л�õģ�
		* @note
		* @param	  Mat& m    1*6//1*10���� �� Ԫ��Ϊ�� x,y,z,rx,ry,rz  or x,y,z, q0,q1,q2,q3,rx,ry,rz
		* @param	  bool useQuaternion      ԭʼ�����Ƿ�ʹ����Ԫ����ʾ
		* @param	  string& seq         ԭʼ����ʹ��ŷ���Ǳ�ʾʱ������ϵ����ת˳��
		* @return	  ����ת�������ξ���
		**************************************************/
		virtual cv::Mat attitudeVectorToMatrix(const cv::Mat& m, bool useQuaternion, const std::string& seq) = 0;

		/**************************************************
		* @brief	����Ƿ�����ת����
		* @note
		* @param
		* @param
		* @param
		* @return  true : ����ת���� false : ������ת����
		**************************************************/
		virtual bool isRotatedMatrix(cv::Mat& R) = 0;

		/**************************************************
		* @brief   ����Ԫ��ת��Ϊ��ת����
		* @note
		* @param   const Vec4d& q   ��һ������Ԫ��: q = q0 + q1 * i + q2 * j + q3 * k;
		* @return  ����3*3��ת����R
		**************************************************/
		virtual cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q) = 0;

	};
}


