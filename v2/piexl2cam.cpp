#include "pch.h"
#include "BLCalibration.h"


void bl::piexl2Cam(cv::Point2d piexl,cv::Point3f& camPoint, double zc, cv::Mat cameraMatrix) {

	float fx = cameraMatrix.at<float>(0, 0);
	float fy = cameraMatrix.at<float>(1, 1);
	float cx = cameraMatrix.at<float>(0, 2);
	float cy = cameraMatrix.at<float>(1, 2);

	camPoint.x = zc * (piexl.x - cx) / fx;
	camPoint.y = zc * (piexl.y - cy) / fy;
	camPoint.z = zc;

}
