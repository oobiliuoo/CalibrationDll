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
* @brief   欧拉角转换为旋转矩阵
* @note
* @param    const std::string& seq  指定欧拉角的排列顺序；（机械臂的位姿类型有xyz,zyx,zyz几种，需要区分）
* @param    const Mat& eulerAngle   欧拉角（1*3矩阵）, 角度值
* @param
* @return   返回3*3旋转矩阵
**************************************************/
_declspec(dllexport)  cv::Mat eulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const std::string& seq);
 
/**************************************************
* @brief      将采集的原始数据转换为齐次矩阵（从机器人控制器中获得的）
* @note
* @param	  Mat& m    1*6//1*10矩阵 ， 元素为： x,y,z,rx,ry,rz  or x,y,z, q0,q1,q2,q3,rx,ry,rz
* @param	  bool useQuaternion      原始数据是否使用四元数表示
* @param	  string& seq         原始数据使用欧拉角表示时，坐标系的旋转顺序
* @return	  返回转换完的齐次矩阵
**************************************************/
_declspec(dllexport) cv::Mat attitudeVectorToMatrix(const cv::Mat& m, bool useQuaternion, const std::string& seq);
