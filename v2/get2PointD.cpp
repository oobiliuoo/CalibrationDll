#include "pch.h"
#include "BLCalibration.h"

double bl::get2PointD(cv::Point3f p1,cv::Point3f p2)
{
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}