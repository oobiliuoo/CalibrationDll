#pragma once
#include "AbsCalibration.h"
#include "AbsUtils.h"
#include "UtilsFactory.h"

class CLASS_DECLSPEC ToolCalibration : public bl::AbsToolCalibration
{


public:

	ToolCalibration();
	~ToolCalibration();

	int mType();

	int stereoToolRepair(cv::Mat imgL, cv::Mat imgR, cv::Mat endPos, cv::Mat H_cam2base_L,
		cv::Mat intrinsic_L, cv::Mat intrinsic_R, cv::Mat H_left2right,
		cv::Mat& T_tool2end, int cout_flog = 0, cv::Rect rect = cv::Rect(0, 0, 0, 0));

	int stereoToolRepair(cv::Mat imgL, cv::Mat imgR, cv::Mat endPos);

	int stereoToolRepair(cv::Point2f point_L, cv::Point2f point_R, cv::Mat endPos);

	cv::Point2d Cusp(cv::Mat src, int srcshow);
	cv::Point2d Cusp2(cv::Mat src, int threshold_data);

	cv::Point3f stereoPiexl2Obj(cv::Point2f& piexl_l, cv::Point2f& piexl_r);

	void Tool2End(cv::Mat j6Pos, cv::Point3d toolPoint);

	void setIntrinsic_l(cv::Mat src);
	void setIntrinsic_r(cv::Mat src);
	void setH_left2right(cv::Mat src);
	void setH_cam2obj_l(cv::Mat src);
	cv::Mat getT_tool2end();
	void setRect_L(cv::Rect rect);
	void setRect_R(cv::Rect rect);
	void setThreshold(int, int);
	void setCout(int);

public:

	bl::AbsUtilsFactory* fac = NULL;
	bl::AbsTransUtils* util = NULL;




};

