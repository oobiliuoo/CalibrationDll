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
        // ��������ת��һ��ƽ��
        temp.x = (points2D[i].x - cx) / fx;
        temp.y = (points2D[i].y - cy) / fy;
        temp.z = 1;

        // ��һ������ѧ����
        double norm = sqrt(temp.x * temp.x + temp.x * temp.x + 1);
        temp.z = 1.0 / norm;
        temp.x /= temp.z;
        temp.y /= temp.z;

        imgPoints.push_back(temp);

    }

    // ��������ϵ�����߾���
    double distances[3];
    distances[0] = bl::get2PointD(points3D[1], points3D[2]); // BC
    distances[1] = bl::get2PointD(points3D[0], points3D[2]); // AC
    distances[2] = bl::get2PointD(points3D[0], points3D[1]); // AB
    std::cout << "distance: " << distances[0] << " " << distances[1] << " " << distances[2] << std::endl;

    

    // ��һ��ƽ�����߾���
    double BC_norm = bl::get2PointD(imgPoints[1], imgPoints[2]);
    double AC_norm = bl::get2PointD(imgPoints[0], imgPoints[2]); // AC
    double AB_norm = bl::get2PointD(imgPoints[0], imgPoints[1]); // AB
   
    // ԭ�㵽��һ��ƽ���������
    cv::Point3f o(0, 0, 0);
    double PA_norm = bl::get2PointD(imgPoints[0], o);
    double PB_norm = bl::get2PointD(imgPoints[1], o);
    double PC_norm = bl::get2PointD(imgPoints[2], o);

    // ������֮��ĽǶ�ֵ

    double cosines[3];
    //cosines[0] = imgPoints[1].x * imgPoints[2].x + imgPoints[1].y * imgPoints[2].y + imgPoints[1].z * imgPoints[2].z; // ��BPC
    //cosines[1] = imgPoints[0].x * imgPoints[2].x + imgPoints[0].y * imgPoints[2].y + imgPoints[0].z * imgPoints[2].z; // ��APC
    //cosines[2] = imgPoints[0].x * imgPoints[1].x + imgPoints[0].y * imgPoints[1].y + imgPoints[0].z * imgPoints[1].z; // ��APB
    cosines[0] = bl::getCosines(PB_norm,PC_norm,BC_norm);
    cosines[1] = bl::getCosines(PA_norm,PC_norm,AC_norm);
    cosines[2] = bl::getCosines(PA_norm,PB_norm,AB_norm);

    std::cout << "cosines: " << cosines[0] << " " << cosines[1] << " " << cosines[2] << std::endl;

    // ����Ԫ�����PA,PB,PC��ֵ���������
    double lengths[4][3];
    int n = solveForLengths(lengths, distances, cosines);

    cv::Point3f camPoints[3];

    for (int i = 0; i < 1; i++)
    {
        camPoints[0].x = lengths[i][0] * imgPoints[0].x;
        camPoints[0].y = lengths[i][0] * imgPoints[0].y;
        camPoints[0].z = lengths[i][0] * imgPoints[0].z;

        camPoints[1].x = lengths[i][1] * imgPoints[1].x;
        camPoints[1].y = lengths[i][1] * imgPoints[1].y;
        camPoints[1].z = lengths[i][1] * imgPoints[1].z;

        camPoints[2].x = lengths[i][2] * imgPoints[2].x;
        camPoints[2].y = lengths[i][2] * imgPoints[2].y;
        camPoints[2].z = lengths[i][2] * imgPoints[2].z;

        std::cout << "cam A: " << camPoints[0]<<std::endl;
        std::cout << "cam B: " << camPoints[1]<<std::endl;
        std::cout << "cam C: " << camPoints[2]<<std::endl<<std::endl;


        // ICP ��RT
        bl::vP3F camPs;
        camPs.push_back(camPoints[0]);
        camPs.push_back(camPoints[1]);
        camPs.push_back(camPoints[2]);

        bl::ICP(camPs,points3D,R,T);

    }
        

}
