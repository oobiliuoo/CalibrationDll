#include "pch.h"
#include "BLCalibration.h"


double stringToNum(const std::string& str) {
    std::istringstream iss(str);
    double num;
    iss >> num;
    return num;
}

cv::Mat_<double>  bl::analyzePose(char* bufRecv, int size, cv::Mat_<double> ToolPose)
{
    std::string tmp;
    cv::Mat ToolPose_tmp = cv::Mat::zeros(1, 6, CV_64F);
    for (int i = 0, j = 0; i < size; ++i) {
        if (' ' == bufRecv[i]) {
            ToolPose_tmp.at<double>(0, j) = stringToNum(tmp);
            tmp = "";
            j++;
        }
        else {
            tmp = tmp + bufRecv[i];
            if (i == size - 1) {
                ToolPose_tmp.at<double>(0, j) = stringToNum(tmp);
                //cout << tmp << endl;
            }
        }
    }

    if (ToolPose.empty()) {
        ToolPose_tmp.copyTo(ToolPose);
    }
    else {
        cv::vconcat(ToolPose, ToolPose_tmp, ToolPose);
    }

   // std::cout << "analyzePose_ToolPose = " << ToolPose << std::endl;

    return ToolPose;
}



void bl::writeRobotPos(cv::Mat_<double> ToolPose, std::string robot_pos_name)
{
    std::ofstream txtfile;
    txtfile.open(robot_pos_name);
    for (int i = 0; i < ToolPose.rows; i++) {
        for (int j = 0; j < 6; j++) {
            txtfile << std::setprecision(20) << ToolPose.at<double>(i, j);
            txtfile << " ";
        }
        txtfile << "\n";
    }
    txtfile.close();

}


cv::Mat_<double> bl::readRobotPos(std::string file,int size)
{
    cv::Mat_<double> ToolPose = cv::Mat(size, 6, CV_64FC1);

    //从文件读取位姿矩阵
    std::ifstream txtfile(file);
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < 6; j++) {
            txtfile >> ToolPose.at<double>(i, j);
        }
    }
    txtfile.close();

    return ToolPose;
}

