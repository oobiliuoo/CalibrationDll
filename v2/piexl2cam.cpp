#include "pch.h"
#include "BLCalibration.h"


void bl::piexl2Cam(cv::Point2d piexl,cv::Point3d& camPoint, double zc, cv::Mat cameraMatrix) {

	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double cx = cameraMatrix.at<double>(0, 2);
	double cy = cameraMatrix.at<double>(1, 2);

	camPoint.x = zc * (piexl.x - cx) / fx;
	camPoint.y = zc * (piexl.y - cy) / fy;
	camPoint.z = zc;

}
