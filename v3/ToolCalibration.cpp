#include "ToolCalibration.h"

ToolCalibration::ToolCalibration()
{

	fac = new UtilsFactory();
	this->util = fac->getTransUtils();

	this->rect_L = cv::Rect(0, 0, 0, 0);
	this->rect_R = cv::Rect(0, 0, 0, 0);

}

ToolCalibration::~ToolCalibration()
{
	fac->deleteObject(this->util);
	fac->close();
}


int ToolCalibration::mType()
{
	return 2;
}

void ToolCalibration::setIntrinsic_l(cv::Mat src)
{
	if (src.empty())
	{
		std::cout << "Intrinsic_l is null\n";
		return;
	}
	this->util->to_CV_32F(src);
	this->intrinsic_L = src.clone();
}

void ToolCalibration::setIntrinsic_r(cv::Mat src)
{
	if (src.empty())
	{
		std::cout << "intrinsic_r is null\n";
		return;
	}
	this->util->to_CV_32F(src);
	this->intrinsic_R = src.clone();
}

void ToolCalibration::setH_left2right(cv::Mat src)
{
	if (src.empty())
	{
		std::cout << "H_left2right is null\n";
		return;
	}
	this->util->to_CV_32F(src);
	this->H_left2right = src.clone();
}

void ToolCalibration::setH_cam2obj_l(cv::Mat src)
{
	if (src.empty())
	{
		std::cout << "H_cam2obj_l is null\n";
		return;
	}
	this->util->to_CV_32F(src);
	this->H_cam2obj_L = src.clone();

}

void ToolCalibration::setRect_L(cv::Rect rect)
{
	if (rect.empty())
	{
		std::cout << "rect_l is null\n";
		return;
	}
	this->rect_L = rect;
}

void ToolCalibration::setRect_R(cv::Rect rect)
{
	if (rect.empty())
	{
		std::cout << "rect_r is null\n";
		return;
	}
	this->rect_R = rect;
}

void ToolCalibration::setThreshold(int left, int right)
{
	if (left < 0 || left>255 || right < 0 || right>255)
	{
		std::cout << "the threshold data invalue\n";
		return;
	}

	this->threshold_data[0] = left;
	this->threshold_data[1] = right;

}

void ToolCalibration::setCout(int cout_log)
{
	this->cout_f = cout_log;
}

cv::Mat ToolCalibration::getT_tool2end()
{
	if (this->H_tool2end.empty())
	{
		std::cout << "H_tool2end is null\n";
		return cv::Mat();
	}
	cv::Mat R, T;
	this->util->H2R_T(this->H_tool2end, R, T);
	return T.clone();

}

cv::Point2d ToolCalibration::Cusp(cv::Mat src, int srcshow)
{

	cv::Mat cuspsrc, cuspsrc1, cuspsrc2, cuspsrc3, cuspsrc4;
	medianBlur(src, cuspsrc, 5);
	cvtColor(cuspsrc, cuspsrc1, cv::COLOR_BGR2GRAY);
	threshold(cuspsrc1, cuspsrc2, 30, 255, cv::THRESH_OTSU); //cv::THRESH_BINARY
	threshold(cuspsrc2, cuspsrc2, 30, 255, cv::THRESH_BINARY_INV);
	if (srcshow >= 3) {
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
	if (srcshow >= 3) {
		imshow("labelMap", src_color);
	}


	cuspsrc4 = src_color.clone();
	//cvtColor(cuspsrc4, cuspsrc4, COLOR_BGR2GRAY);
	threshold(cuspsrc4, cuspsrc4, 30, 255, cv::THRESH_OTSU);
	//threshold(cuspsrc4, cuspsrc4, 30, 255, THRESH_BINARY);
	if (srcshow >= 3) {
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
	if (srcshow >= 3) {
		imshow("final", cuspsrc);
	}
	return cuspP;

}

cv::Point2d ToolCalibration::Cusp2(cv::Mat src,int threshold_data)
{
	cv::Mat cuspsrc, cuspsrc1, cuspsrc2, cuspsrc3, cuspsrc4;
	medianBlur(src, cuspsrc, 5);
	cvtColor(cuspsrc, cuspsrc1, cv::COLOR_BGR2GRAY);
	threshold(cuspsrc1, cuspsrc2, threshold_data, 255, cv::THRESH_BINARY); //cv::THRESH_BINARY
	threshold(cuspsrc2, cuspsrc2, 30, 255, cv::THRESH_BINARY_INV);
	if (this->cout_f >= 3) {
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
	if (this->cout_f >= 3) {
		imshow("labelMap", src_color);
	}


	cuspsrc4 = src_color.clone();
	//cvtColor(cuspsrc4, cuspsrc4, COLOR_BGR2GRAY);
	threshold(cuspsrc4, cuspsrc4, 30, 255, cv::THRESH_OTSU);
	//threshold(cuspsrc4, cuspsrc4, 30, 255, THRESH_BINARY);
	if (this->cout_f >= 3) {
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
	if (this->cout_f >= 3) {
		imshow("final", cuspsrc);
	}
	return cuspP;


}

cv::Point3f ToolCalibration::stereoPiexl2Obj(cv::Point2f& piexl_l, cv::Point2f& piexl_r)
{


	//  [u1]      |X|					  [u2]      |X|
	//Z*|v1| = Ml*|Y|					Z*|v2| = Mr*|Y|
	//  [ 1]      |Z|					  [ 1]      |Z|
	//			  |1|								|1|


	cv::Mat h_obj2cam_l = this->H_cam2obj_L.inv();
	cv::Mat h_obj2cam_r = this->H_left2right * h_obj2cam_l;

	cv::Mat rt_l = h_obj2cam_l(cv::Rect(0, 0, 4, 3));
	cv::Mat rt_r = h_obj2cam_r(cv::Rect(0, 0, 4, 3));





	cv::Mat M_l = this->intrinsic_L * rt_l;
	cv::Mat M_r = this->intrinsic_R * rt_r;

	if (this->cout_f >= 4)
	{
		std::cout << "rt_1\n" << rt_l << std::endl;
		std::cout << "rt_r\n" << rt_r << std::endl;
		std::cout << "intrinsic_l\n" << intrinsic_L << std::endl;
		std::cout << "intrinsic_r\n" << intrinsic_R << std::endl;
		std::cout << "piexl_l\n" << piexl_l << std::endl;
		std::cout << "piexl_r\n" << piexl_r << std::endl;
		std::cout << "左相机M矩阵 =\n " << std::endl << M_l << std::endl;
		std::cout << "右相机M矩阵 =\n " << std::endl << M_r << std::endl;
	}
	


		//最小二乘法A矩阵
	cv::Mat A = cv::Mat(4, 3, CV_32F);
	A.at<float>(0, 0) = piexl_l.x * M_l.at<float>(2, 0) - M_l.at<float>(0, 0);
	A.at<float>(0, 1) = piexl_l.x * M_l.at<float>(2, 1) - M_l.at<float>(0, 1);
	A.at<float>(0, 2) = piexl_l.x * M_l.at<float>(2, 2) - M_l.at<float>(0, 2);

	A.at<float>(1, 0) = piexl_l.y * M_l.at<float>(2, 0) - M_l.at<float>(1, 0);
	A.at<float>(1, 1) = piexl_l.y * M_l.at<float>(2, 1) - M_l.at<float>(1, 1);
	A.at<float>(1, 2) = piexl_l.y * M_l.at<float>(2, 2) - M_l.at<float>(1, 2);

	A.at<float>(2, 0) = piexl_r.x * M_r.at<float>(2, 0) - M_r.at<float>(0, 0);
	A.at<float>(2, 1) = piexl_r.x * M_r.at<float>(2, 1) - M_r.at<float>(0, 1);
	A.at<float>(2, 2) = piexl_r.x * M_r.at<float>(2, 2) - M_r.at<float>(0, 2);

	A.at<float>(3, 0) = piexl_r.y * M_r.at<float>(2, 0) - M_r.at<float>(1, 0);
	A.at<float>(3, 1) = piexl_r.y * M_r.at<float>(2, 1) - M_r.at<float>(1, 1);
	A.at<float>(3, 2) = piexl_r.y * M_r.at<float>(2, 2) - M_r.at<float>(1, 2);

	//最小二乘法B矩阵
	cv::Mat B = cv::Mat(4, 1, CV_32F);
	B.at<float>(0, 0) = M_l.at<float>(0, 3) - piexl_l.x * M_l.at<float>(2, 3);
	B.at<float>(1, 0) = M_l.at<float>(1, 3) - piexl_l.y * M_l.at<float>(2, 3);
	B.at<float>(2, 0) = M_r.at<float>(0, 3) - piexl_r.x * M_r.at<float>(2, 3);
	B.at<float>(3, 0) = M_r.at<float>(1, 3) - piexl_r.y * M_r.at<float>(2, 3);

	//	std::cout << "A:\n" << A << std::endl;
	//	std::cout << "B:\n" << B << std::endl;
	cv::Mat XYZ = cv::Mat(3, 1, CV_32F);
	//采用SVD最小二乘法求解XYZ
	solve(A, B, XYZ, cv::DECOMP_SVD);

	//	std::cout<<"空间坐标为 = "<< std::endl<<XYZ<< std::endl;

		//世界坐标系中坐标
	cv::Point3f world;
	world.x = XYZ.at<float>(0, 0);
	world.y = XYZ.at<float>(1, 0);
	world.z = XYZ.at<float>(2, 0);

	return world;

}

void ToolCalibration::Tool2End(cv::Mat j6Pos, cv::Point3d toolPoint)
{
	// 将j6的位姿转成H
	cv::Mat H_j62base = this->util->attitudeVectorToMatrix(j6Pos, false, "xyz");
	cv::Mat R_j6, T_j6;
	this->util->H2R_T(H_j62base, R_j6, T_j6);
	//std::cout << "H_j62base:\n" << H_j62base << std::endl;

	cv::Mat tool_base = (cv::Mat_<double>(4, 1) << toolPoint.x, toolPoint.y, toolPoint.z, 1);
	cv::Mat H_tool2base;
	this->util->R_T2H(R_j6, tool_base, H_tool2base);
	//	std::cout << "H_tool2base:\n" << H_tool2base << std::endl;

	this->H_tool2end = H_j62base.inv() * H_tool2base;
	//	std::cout << "H_tool2j6:\n" << H_tool2j6 << std::endl;
	this->util->to_CV_32F(this->H_tool2end);

}

int ToolCalibration::stereoToolRepair(cv::Mat imgL, cv::Mat imgR, cv::Mat endPos, cv::Mat H_cam2base_L,
	cv::Mat intrinsic_L, cv::Mat intrinsic_R, cv::Mat H_left2right,
	cv::Mat& T_tool2end, int cout_flog, cv::Rect rect)
{

	// 验证数据
	// 验证照片是否合格
	if (!imgL.data || !imgR.data)
	{
		if (cout_flog)
			std::cout << "check the img " << std::endl;
		return 1;
	}
	if (!intrinsic_L.data || !intrinsic_R.data || !H_left2right.data)
	{
		if (cout_flog)
			std::cout << "check the cam " << std::endl;
		return 2;
	}
	// 统一数据类型
	if (intrinsic_L.type() != 5)
		intrinsic_L.convertTo(intrinsic_L, CV_32F);
	if (intrinsic_R.type() != 5)
		intrinsic_R.convertTo(intrinsic_R, CV_32F);
	if (H_left2right.type() != 5)
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

	if (cout_flog == 3)
		cv::waitKey(0);
	tempP = Cusp(imgR, cout_flog);
	p2.x = (float)tempP.x;
	p2.y = (float)tempP.y;

	if (p1.x == 0 || p2.x == 0)
	{
		if (cout_flog)
			std::cout << "can not find the tool" << std::endl;
		return 3;
	}

	if (cout_flog == 2)
	{
		circle(imgL, p1, 1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		circle(imgR, p2, 1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

		cv::imshow("left", imgL);
		cv::imshow("right", imgR);
	}

	if (rect.area() != 0)
	{
		p1.x += rect.x;
		p1.y += rect.y;
		p2.x += rect.x;
		p2.y += rect.y;
	}


	if (cout_flog)
	{
		std::cout << "p1" << p1 << std::endl;
		std::cout << "p2" << p2 << std::endl;
	}



	// 像素点转相机点
	cv::Point3f toolPoint;

	//cv::Mat H_base2cam_L = H_cam2base_L.inv();
//	cv::Mat H_base2cam_R = H_left2right * H_base2cam_L;
	// std::cout << "H2 2 " << H_cam2base_R << std::endl;


	setIntrinsic_l(intrinsic_L);
	setIntrinsic_r(intrinsic_R);
	setH_left2right(H_left2right);
	setH_cam2obj_l(H_cam2base_L);

	toolPoint = stereoPiexl2Obj(p1, p2);
	if (cout_flog)
		std::cout << "toolPoint" << toolPoint << std::endl;

	// 解析位姿
	cv::Mat mEndPos(1, 6, CV_64F);
	cv::hconcat(endPos.rowRange(0, 1), endPos.rowRange(1, 2), mEndPos);


	cv::Point3d baseTool;
	baseTool.x = toolPoint.x;
	baseTool.y = toolPoint.y;
	baseTool.z = toolPoint.z;

	cv::Mat H_tool2end, R;
	// 求工具端
	Tool2End(mEndPos,baseTool);
	//	std::cout << "dll: H_tool2endL\n" << H_tool2end << std::endl;
	cv::Mat t;
	this->util->H2R_T(this->H_tool2end, R, t);
	T_tool2end = t.clone();


	if (cout_flog == 2)
		cv::waitKey(0);

	return 0;

}

int ToolCalibration::stereoToolRepair(cv::Mat imgL, cv::Mat imgR, cv::Mat endPos)
{
	// 验证数据
	// 验证照片是否合格
	if (imgL.empty() || imgR.empty())
	{
		if (cout_f>=1)
			std::cout << "check the img " << std::endl;
		return 1;
	}
	if (this->intrinsic_L.empty() || this->intrinsic_R.empty() || this->H_left2right.empty())
	{
		if (cout_f >= 1)
			std::cout << "check the cam " << std::endl;
		return 2;
	}
	// 统一数据类型


	// 提取像素点
	// 图像处理
	if (this->rect_L.area() != 0)
	{
		imgL = imgL(this->rect_L);
	}
	if (this->rect_R.area() != 0)
	{
		imgR = imgR(this->rect_R);
	}


	// 提取像素点
	cv::Point2d tempP;
	cv::Point2f p1, p2;

	if (this->threshold_data[0] == 0)
		tempP = Cusp(imgL,cout_f);
	else 
		tempP = Cusp2(imgL, this->threshold_data[0]);

	p1.x = (float)tempP.x;
	p1.y = (float)tempP.y;

	if (cout_f >= 3)
		cv::waitKey(0);

	if (this->threshold_data[1] == 0)
		tempP = Cusp(imgR, cout_f);
	else
		tempP = Cusp2(imgR, this->threshold_data[0]);

	p2.x = (float)tempP.x;
	p2.y = (float)tempP.y;

	if (p1.x == 0 || p2.x == 0)
	{
		if (cout_f >= 1)
			std::cout << "can not find the tool" << std::endl;
		return 3;
	}

	if (cout_f >= 2)
	{
		circle(imgL, p1, 1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		circle(imgR, p2, 1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

		cv::imshow("left", imgL);
		cv::imshow("right", imgR);
	}


	if (this->rect_L.area() != 0)
	{
		p1.x += this->rect_L.x;
		p1.y += this->rect_L.y;
	}
	if (this->rect_R.area() != 0)
	{
		p2.x += this->rect_R.x;
		p2.y += this->rect_R.y;
	}


	if (cout_f >= 1)
	{
		std::cout << "p1" << p1 << std::endl;
		std::cout << "p2" << p2 << std::endl;
	}

	stereoToolRepair(p1, p2,endPos);

	return 0;


}

int ToolCalibration::stereoToolRepair(cv::Point2f point_L, cv::Point2f point_R, cv::Mat endPos)
{

	cv::Point3f toolPoint;
	toolPoint = stereoPiexl2Obj(point_L, point_R);
	if (cout_f >= 1)
		std::cout << "toolPoint" << toolPoint << std::endl;

	// 解析位姿
	cv::Mat mEndPos(1, 6, CV_64F);
	cv::hconcat(endPos.rowRange(0, 1), endPos.rowRange(1, 2), mEndPos);


	cv::Point3d baseTool;
	baseTool.x = toolPoint.x;
	baseTool.y = toolPoint.y;
	baseTool.z = toolPoint.z;

	cv::Mat  R;
	// 求工具端
	Tool2End(mEndPos, baseTool);

	//	std::cout << "dll: H_tool2endL\n" << H_tool2end << std::endl;
	cv::Mat t;
	this->util->H2R_T(H_tool2end, R, t);

	if (cout_f >= 2)
		cv::waitKey(0);

	return 0;

}
