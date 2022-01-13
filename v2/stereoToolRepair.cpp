#include "pch.h"
#include "BLCalibration.h"
cv::Point2d Cusp(cv::Mat src, int srcshow);

int bl::stereoToolRepair(cv::Mat imgL, cv::Mat imgR, cv::Mat endPos, cv::Mat H_cam2base_L,
	cv::Mat intrinsic_L, cv::Mat intrinsic_R, cv::Mat H_left2right,
	cv::Mat& T_tool2end,int cout_flog,cv::Rect rect)
{

	// 验证数据
	// 验证照片是否合格
	if (!imgL.data || !imgR.data )
	{
		if(cout_flog)
			std::cout << "check the img "<< std::endl;
		return 1;
	}
	if(!intrinsic_L.data || !intrinsic_R.data || !H_left2right.data)
	{
		if(cout_flog)
			std::cout << "check the cam "<< std::endl;
		return 2;
	}
	// 统一数据类型
	if(intrinsic_L.type()!=5)
		intrinsic_L.convertTo(intrinsic_L, CV_32F);
	if(intrinsic_R.type()!=5)
		intrinsic_R.convertTo(intrinsic_R, CV_32F);
	if(H_left2right.type()!=5)
		H_left2right.convertTo(H_left2right, CV_32F);
	if (H_cam2base_L.type() != 5)
		H_cam2base_L.convertTo(H_cam2base_L, CV_32F);


	// 提取像素点
	// 图像处理
	if (rect.area() != 0)
	{
		imgL = imgL(rect);
		imgR = imgR(rect);
	}

	// 提取像素点
	cv::Point2d tempP;
	cv::Point2f p1, p2;

	tempP = Cusp(imgL, cout_flog);
	p1.x = (float)tempP.x;
	p1.y = (float)tempP.y;

	tempP = Cusp(imgR, cout_flog);
	p2.x = (float)tempP.x;
	p2.y = (float)tempP.y;

	if (p1.x == 0 || p2.x == 0)
	{
		if (cout_flog)
			std::cout << "can not find the tool" << std::endl;
		return 3;
	}

	if (rect.area() != 0)
	{
		p1.x += rect.x;
		p1.y += rect.y;
		p2.x += rect.x;
		p2.y += rect.y;
	}


	if(cout_flog)
	{
		std::cout << "p1" << p1 << std::endl;
		std::cout << "p2" << p2 << std::endl;
	}

	// 像素点转相机点
	cv::Point3f toolPoint;

	cv::Mat H_base2cam_L = H_cam2base_L.inv();
	cv::Mat H_base2cam_R = H_left2right * H_base2cam_L;
	// std::cout << "H2 2 " << H_cam2base_R << std::endl;

	toolPoint = bl::stereoPiexl2Cam(p1, p2, intrinsic_L, intrinsic_R, H_base2cam_L, H_base2cam_R);
	if(cout_flog)
		std::cout << "toolPoint" << toolPoint << std::endl;

	// 解析位姿
	cv::Mat mEndPos(1, 6, CV_64F);
	cv::hconcat(endPos.rowRange(0, 1), endPos.rowRange(1, 2), mEndPos);


	cv::Point3d baseTool;
	baseTool.x = toolPoint.x;
	baseTool.y = toolPoint.y;
	baseTool.z = toolPoint.z;

	cv::Mat H_tool2end,R;
	// 求工具端
	bl::Tool2J6(mEndPos,baseTool,H_tool2end);

//	std::cout << "dll: H_tool2endL\n" << H_tool2end << std::endl;
	cv::Mat t;
	bl::H2R_T(H_tool2end, R, t);
	T_tool2end = t.clone();


	return 0;

}


cv::Point2d Cusp(cv::Mat src, int srcshow) {
	cv::Mat cuspsrc, cuspsrc1, cuspsrc2, cuspsrc3, cuspsrc4;
	medianBlur(src, cuspsrc, 5);
	cvtColor(cuspsrc, cuspsrc1, cv::COLOR_BGR2GRAY);
	threshold(cuspsrc1, cuspsrc2, 30, 255, cv::THRESH_OTSU);
	threshold(cuspsrc2, cuspsrc2, 30, 255, cv::THRESH_BINARY_INV);
	if (srcshow == 10086) {
		imshow("阈值", cuspsrc2);
	}

	cuspsrc3 = cuspsrc2.clone();
	cv::Mat src_color, g_src, labels, stats, centroids;
	int num = connectedComponentsWithStats(cuspsrc3, labels, stats, centroids);
	for (int i = 1; i < num; i++) {
		int area = stats.at<int>(i, cv::CC_STAT_AREA);
		//std::cout << "count:" << i << "  area:" << area << std::endl;
	}
	//vector<Vec3b> color(num+1 );
	//color[0] = Vec3b(0, 0, 0);//背景色
	std::vector<int> color(num + 1);
	int areatemp = 0, mtemp = 0;
	for (int m = 1; m < num; m++) {
		int area = stats.at<int>(m, cv::CC_STAT_AREA);
		if (area > areatemp) {
			areatemp = area;
			mtemp = m;
		}

	}

	for (int m = 1; m <= num; m++) {
		if (m == mtemp) {
			color[m] = 255;
		}
		else {
			color[m] = 0;
		}
	}
	src_color = cv::Mat::zeros(src.size(), CV_8UC1);
	for (int x = 0; x < src.rows; x++)
		for (int y = 0; y < src.cols; y++)
		{
			int label = labels.at<int>(x, y);//注意labels是int型，不是uchar.
			src_color.at<uchar>(x, y) = color[label];
		}
	if (srcshow == 10086) {
		imshow("labelMap", src_color);
	}


	cuspsrc4 = src_color.clone();
	//cvtColor(cuspsrc4, cuspsrc4, COLOR_BGR2GRAY);
	threshold(cuspsrc4, cuspsrc4, 30, 255, cv::THRESH_OTSU);
	//threshold(cuspsrc4, cuspsrc4, 30, 255, THRESH_BINARY);
	if (srcshow == 10086) {
		imshow("4444", cuspsrc4);
	}
	std::vector<cv::Point2d> cuspPv;
	int sump = 0;
	cv::Point2d cuspP, cuspP2;
	int i, j;
	for (i = cuspsrc4.rows - 1; i >= 0; i--) {
		for (j = 0; j < cuspsrc4.cols - 5; ++j) {

			if (cuspsrc4.at<uchar>(i, j) == 255) {
				sump++;
				cuspP.x = j;
				cuspP.y = i;
				cuspPv.push_back(cuspP);
				//cout <<"sump1=" << sump << endl;
				//break;
			}
		}
		if (sump != 0) {
			if (sump % 2 == 0) {
				sump = sump / 2 - 1;
				//cout << "sump2=" << sump << endl;
			}
			else {
				sump = (sump - 1) / 2;
				//cout << "sump3=" << sump << endl;
			}
			//cout << "sump4=" << sump << endl;
			//cout << cuspPv.size() << endl;
			cuspP2 = cuspPv[sump];
			cuspP2 = cuspPv[sump];
			break;
		}
	}
	circle(cuspsrc, cuspP2, 1, cv::Scalar(0, 0, 255), 8, cv::LINE_AA);
	if (srcshow == 10086) {
		imshow("final", cuspsrc);
	}
	return cuspP;
}


