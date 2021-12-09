#include "pch.h"
#include "BLCalibration.h"

int bl::dealP3P(std::vector<cv::Point3f> points3D, std::vector<cv::Point2f> points2D,
	const cv::Mat cameraMatrix, const cv::Mat distCoeffs, cv::Mat& R, cv::Mat& T, int method)
{

	// ����У��
    if (cameraMatrix.cols == 0 || distCoeffs.cols == 0)
    {
        std::cout<<"Err:-1,����ڲ�����������δ����!\n";
        return -1;
    }

    if (points3D.size() != points2D.size())
    {
        std::cout<<"Err:-2��3D��������2D��������һ��\n";
        return -2;
    }


    if (method == SOLVEPNP_P3P || method == SOLVEPNP_ITERATIVE)
    {
        if (points3D.size() != 4)
        {
            std::cout << "Err:-3,ʹ�÷���0�ͷ���2ʱ����ĵ���ӦΪ4��\n";
            return -3;
        }
    }
    else
    {
        if (points3D.size() < 4)
        {
            printf("Err:-4,����ĵ���Ӧ����4��\n");
            return -4;
        }
    }


    // ��������
    int iterationsCount = 300;      
    // ��ͶӰ��Χ
    float reprojectionError = 5.991; 
    // ��������
    double confidence = 0.95;        

    cv::Mat inliers;
  
    // ����opencv����
    cv::solvePnPRansac(points3D,points2D,cameraMatrix,distCoeffs,R,T,false,iterationsCount,reprojectionError,confidence,inliers,method);

  //  cv::solvePnP(points3D,points2D,cameraMatrix,distCoeffs,R,T,false,method);
	Rodrigues(R, R);
	R.convertTo(R, CV_32FC1);
	T.convertTo(T, CV_32FC1);

}
