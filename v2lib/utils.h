#pragma once
#include "BLCalibration.h"

void blP3P(bl::vP3D points3D, bl::vP2F points2D, const cv::Mat cameraMatrix, const cv::Mat distCoeffs, cv::Mat& R, cv::Mat& T);

int solveForLengths(double lengths[4][3], double distances[3], double cosines[3]);

_declspec(dllexport) bool align(double M_end[3][3],
    double X0, double Y0, double Z0,
    double X1, double Y1, double Z1,
    double X2, double Y2, double Z2,
    double R[3][3], double T[3]);


/**************************************************
* @brief   ŷ����ת��Ϊ��ת����
* @note
* @param    const std::string& seq  ָ��ŷ���ǵ�����˳�򣻣���е�۵�λ��������xyz,zyx,zyz���֣���Ҫ���֣�
* @param    const Mat& eulerAngle   ŷ���ǣ�1*3����, �Ƕ�ֵ
* @param
* @return   ����3*3��ת����
**************************************************/
_declspec(dllexport)  cv::Mat eulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const std::string& seq);
 
/**************************************************
* @brief      ���ɼ���ԭʼ����ת��Ϊ��ξ��󣨴ӻ����˿������л�õģ�
* @note
* @param	  Mat& m    1*6//1*10���� �� Ԫ��Ϊ�� x,y,z,rx,ry,rz  or x,y,z, q0,q1,q2,q3,rx,ry,rz
* @param	  bool useQuaternion      ԭʼ�����Ƿ�ʹ����Ԫ����ʾ
* @param	  string& seq         ԭʼ����ʹ��ŷ���Ǳ�ʾʱ������ϵ����ת˳��
* @return	  ����ת�������ξ���
**************************************************/
_declspec(dllexport) cv::Mat attitudeVectorToMatrix(const cv::Mat& m, bool useQuaternion, const std::string& seq);
