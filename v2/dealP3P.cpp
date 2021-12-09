#include "pch.h"
#include "BLCalibration.h"

int bl::dealP3P(std::vector<cv::Point3f> points3D, std::vector<cv::Point2f> points2D,
	const cv::Mat cameraMatrix, const cv::Mat distCoeffs, cv::Mat& R, cv::Mat& T, int method)
{

	// 数据校验
    if (cameraMatrix.cols == 0 || distCoeffs.cols == 0)
    {
        std::cout<<"Err:-1,相机内参数或畸变参数未设置!\n";
        return -1;
    }

    if (points3D.size() != points2D.size())
    {
        std::cout<<"Err:-2，3D点数量与2D点数量不一致\n";
        return -2;
    }


    if (method == SOLVEPNP_P3P || method == SOLVEPNP_ITERATIVE)
    {
        if (points3D.size() != 4)
        {
            std::cout << "Err:-3,使用方法0和方法2时输入的点数应为4！\n";
            return -3;
        }
    }
    else
    {
        if (points3D.size() < 4)
        {
            printf("Err:-4,输入的点数应大于4！\n");
            return -4;
        }
    }


    // 迭代次数
    int iterationsCount = 300;      
    // 重投影误差范围
    float reprojectionError = 5.991; 
    // 迭代精度
    double confidence = 0.95;        

    cv::Mat inliers;
  
    // 调用opencv函数
    cv::solvePnPRansac(points3D,points2D,cameraMatrix,distCoeffs,R,T,false,iterationsCount,reprojectionError,confidence,inliers,method);

  //  cv::solvePnP(points3D,points2D,cameraMatrix,distCoeffs,R,T,false,method);
	Rodrigues(R, R);
	R.convertTo(R, CV_32FC1);
	T.convertTo(T, CV_32FC1);

}
