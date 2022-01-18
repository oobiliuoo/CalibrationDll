
#include "AbsFactory.h"
#include "CalibrationFactory.h"


bl::AbsCaliFactory* fac = NULL;

void test1()
{
	bl::AbsCamCalibration* cam = NULL;

	fac = new CalibrationFactory();


	cam = fac->getCamCal();

	cv::Mat c = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 摄像机内参数矩阵 */
	cv::Mat k = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	std::string file = "eye2HandCalImageName.txt";

	
	cam->monocularCalbration(file,c,k);

	fac->deleteObject(cam);
	fac->close();

}

void test2()
{ 
	bl::AbsToolCalibration* tool = NULL;

	fac = new CalibrationFactory();
	tool = fac->getToolCal();
	

	cv::Mat c1 = (cv::Mat_<double>(3, 3) << 1573.0294, 0, 618.3772,
	0, 1572.9812, 517.71533,
	0, 0, 1);
	
	cv::Mat c2 = (cv::Mat_<double>(3, 3) << 1568.0557, 0, 620.65314,
	0, 1567.5425, 522.14337,
	0, 0, 1);


	cv::Mat H1 = (cv::Mat_<double>(4, 4) << 0.76430476, -0.0039057897, 0.64484334, -112.16098,
		0.0027469308, 0.99999231, 0.0028010947, -0.37116399,
		-0.6448493, -0.0003695499, 0.76430964, 40.458298,
		0, 0, 0, 1);

	cv::Mat H2 = (cv::Mat_<double>(4, 4) << 0.3395881771481233, 0.005944757987128069, -0.9405554368529744, 1334.747413576235,
	0.9402537840335861, 0.02395473355011668, 0.33963067050999, -246.9658887914438,
	0.02454977702011452, -0.9996953688964542, 0.002545163107792181, 643.4215878951627,
	0, 0, 0, 1);

	cv::Mat imgL = cv::imread("../img/left.png");
	cv::Mat imgR = cv::imread("../img/right.png");

//	cv::imshow("l", imgL);
//	cv::waitKey(0);

	cv::Mat pos = (cv::Mat_<double>(2, 3) << 1069.15, -259.64, 815.98, 157.40, 10.11, 177.83);

	tool->setIntrinsic_l(c1);
	tool->setIntrinsic_r(c2);
	tool->setH_left2right(H1);
	tool->setH_cam2obj_l(H2);

	cv::Mat t;
	tool->stereoToolRepair(imgL,imgR,pos,H2,c1,c2,H1,t,2, cv::Rect(200, 200, 600, 500));

	std::cout << "t\n" << t << std::endl;

	fac->deleteObject(tool);
	fac->close();
}

void test3()
{
	bl::AbsToolCalibration* tool = NULL;

	fac = new CalibrationFactory();
	tool = fac->getToolCal();


	cv::Mat c1 = (cv::Mat_<double>(3, 3) << 1573.0294, 0, 618.3772,
		0, 1572.9812, 517.71533,
		0, 0, 1);

	cv::Mat c2 = (cv::Mat_<double>(3, 3) << 1568.0557, 0, 620.65314,
		0, 1567.5425, 522.14337,
		0, 0, 1);


	cv::Mat H1 = (cv::Mat_<double>(4, 4) << 0.76430476, -0.0039057897, 0.64484334, -112.16098,
		0.0027469308, 0.99999231, 0.0028010947, -0.37116399,
		-0.6448493, -0.0003695499, 0.76430964, 40.458298,
		0, 0, 0, 1);

	cv::Mat H2 = (cv::Mat_<double>(4, 4) << 0.3395881771481233, 0.005944757987128069, -0.9405554368529744, 1334.747413576235,
		0.9402537840335861, 0.02395473355011668, 0.33963067050999, -246.9658887914438,
		0.02454977702011452, -0.9996953688964542, 0.002545163107792181, 643.4215878951627,
		0, 0, 0, 1);

	cv::Mat imgL = cv::imread("../img/left.png");
	cv::Mat imgR = cv::imread("../img/right.png");

	//	cv::imshow("l", imgL);
	//	cv::waitKey(0);

	cv::Mat pos = (cv::Mat_<double>(2, 3) << 1069.15, -259.64, 815.98, 157.40, 10.11, 177.83);

	cv::Rect rect = cv::Rect(200, 200, 600, 500);
	cv::Rect rect1 = cv::Rect(600, 300, 300, 300);
	
	tool->setIntrinsic_l(c1);
	tool->setIntrinsic_r(c2);
	tool->setH_left2right(H1);
	tool->setH_cam2obj_l(H2);
	tool->setRect_L(rect);
	tool->setRect_R(rect1);
	tool->setThreshold(30, 60);
	tool->setCout(2);
	tool->stereoToolRepair(imgL, imgR, pos);
	std::cout << "t\n" << tool->getT_tool2end() << std::endl;

	fac->deleteObject(tool);
	fac->close();
}

int main()
{
	test3();

	return 0;
}