#include "pch.h"
#include "BLCalibration.h"
#include "utils.h"

void blP3P(bl::vP3F points3D, bl::vP2F points2D, const cv::Mat cameraMatrix, const cv::Mat distCoeffs, cv::Mat& R, cv::Mat& T)
{


    float fx = cameraMatrix.at<float>(0, 0);
    float fy = cameraMatrix.at<float>(1, 1);
    float cx = cameraMatrix.at<float>(0, 2);
    float cy = cameraMatrix.at<float>(1, 2);

    bl::vP3F imgPoints;

    for (int i = 0; i < 3; i++)
    {
        cv::Point3d temp;
        // 像素坐标转归一化平面
        temp.x = (points2D[i].x - cx) / fx;
        temp.y = (points2D[i].y - cy) / fy;
        temp.z = 1;

        // 归一化到数学坐标
        double norm = sqrt(temp.x * temp.x + temp.x * temp.x + 1);
        temp.z = 1.0 / norm;
        temp.x /= temp.z;
        temp.y /= temp.z;

        imgPoints.push_back(temp);

    }

    // 世界坐标系下三边距离
    double distances[3];
    distances[0] = bl::get2PointD(points3D[1], points3D[2]); // BC
    distances[1] = bl::get2PointD(points3D[0], points3D[2]); // AC
    distances[2] = bl::get2PointD(points3D[0], points3D[1]); // AB


    // 三个点之间的角度值
    double cosines[3];
    cosines[0] = imgPoints[1].x * imgPoints[2].x + imgPoints[1].y * imgPoints[2].y + imgPoints[1].z * imgPoints[2].z; // 角BPC
    cosines[1] = imgPoints[0].x * imgPoints[2].x + imgPoints[0].y * imgPoints[2].y + imgPoints[0].z * imgPoints[2].z; // 角APC
    cosines[2] = imgPoints[0].x * imgPoints[1].x + imgPoints[0].y * imgPoints[1].y + imgPoints[0].z * imgPoints[1].z; // 角APB

    // 吴消元法求解PA,PB,PC的值，有四组解
    double lengths[4][3];
    int n = solveForLengths(lengths, distances, cosines);

    cv::Point3f camPoints[4];

    for (int i = 0; i < n; i++)
    {
        camPoints[0].x = lengths[i][0] * imgPoints[i].x;
        camPoints[0].y = lengths[i][0] * imgPoints[i].y;
        camPoints[0].z = lengths[i][0] * imgPoints[i].z;

        camPoints[1].x = lengths[i][1] * imgPoints[i].x;
        camPoints[1].y = lengths[i][1] * imgPoints[i].y;
        camPoints[1].z = lengths[i][1] * imgPoints[i].z;

        camPoints[2].x = lengths[i][2] * imgPoints[i].x;
        camPoints[2].y = lengths[i][2] * imgPoints[i].y;
        camPoints[2].z = lengths[i][2] * imgPoints[i].z;

        std::cout << "cam: " << camPoints[0]<<std::endl;
        std::cout << "cam: " << camPoints[1]<<std::endl;
        std::cout << "cam: " << camPoints[2]<<std::endl;

    }
        

}
