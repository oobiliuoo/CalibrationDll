#include "pch.h"
#include "BLCalibration.h"


void calErr(cv::Mat h, bl::vP3D& ori, bl::vP3D tar, double& err);


void bl::ICP(vP3D originPoints, vP3D targetPoints, cv::Mat& R, cv::Mat& T)
{

	// 1.参数设置
	// 错误率
	double err = 100.0;
	// 最高错误率
	double maxErr = 1.0;
	// 迭代次数 
	int maxIter = 100;
	// 设定初始R、T、h
	cv::Mat r_o2t = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	cv::Mat t_o2t = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	cv::Mat h_o2t;
	bl::R_T2H(r_o2t, t_o2t, h_o2t);
	cv::Mat H_final = h_o2t.clone();

	size_t pointSize = originPoints.size();


	int iters = 0;
	while (err > maxErr && iters < maxIter)
	{

		iters++;
		double lastErr = err;

		// 计算误差
		calErr(h_o2t, originPoints, targetPoints, err);

		if (abs(err - lastErr) < 0.1 )
			break;

		// 质心
		cv::Point3d oriCenterP(0.0,0.0,0.0);
		cv::Point3d tarCenterP(0.0,0.0,0.0);

		for (size_t i = 0; i < pointSize; i++)
		{
			oriCenterP.x += originPoints[i].x;
			oriCenterP.y += originPoints[i].y;
			oriCenterP.z += originPoints[i].z;

			tarCenterP.x += targetPoints[i].x;
			tarCenterP.y += targetPoints[i].y;
			tarCenterP.z += targetPoints[i].z;
		}
		
		oriCenterP.x /= pointSize;
		oriCenterP.y /= pointSize;
		oriCenterP.z /= pointSize;

		tarCenterP.x /= pointSize;
		tarCenterP.y /= pointSize;
		tarCenterP.z /= pointSize;

	//	std::cout << "centP:" << oriCenterP << "\t" << tarCenterP << std::endl;
	
		cv::Mat oriMat;
		oriMat = cv::Mat::zeros(3, pointSize, CV_64F);
		cv::Mat tarMat;
		tarMat = cv::Mat::zeros(3, pointSize, CV_64F);

		for (size_t i = 0; i < pointSize; i++)
		{
			oriMat.at<double>(0, i) = originPoints[i].x - oriCenterP.x;
			oriMat.at<double>(1, i) = originPoints[i].y - oriCenterP.y;
			oriMat.at<double>(2, i) = originPoints[i].z - oriCenterP.z;

			tarMat.at<double>(0, i) = targetPoints[i].x - tarCenterP.x;
			tarMat.at<double>(1, i) = targetPoints[i].y - tarCenterP.y;
			tarMat.at<double>(2, i) = targetPoints[i].z - tarCenterP.z;
		
		}

		cv::Mat matH = oriMat * tarMat.t();

		cv::Mat matU, matW, matV;
		cv::SVDecomp(matH, matW, matU, matV);

		cv::Mat matTemp = matU * matV;
		double det = cv::determinant(matTemp);

		double datM[] = { 1, 0, 0, 0, 1, 0, 0, 0, det };
		cv::Mat matS(3, 3, CV_64FC1, datM);
	//	std::cout << "mats:" << matS << std::endl;

		r_o2t = matV.t() * matS * matU.t();
		
		double tx, ty, tz;
		tx = tarCenterP.x - (oriCenterP.x * r_o2t.at<double>(0, 0) + oriCenterP.y * r_o2t.at<double>(0, 1) + oriCenterP.z * r_o2t.at<double>(0, 2));
		ty = tarCenterP.y - (oriCenterP.x * r_o2t.at<double>(1, 0) + oriCenterP.y * r_o2t.at<double>(1, 1) + oriCenterP.z * r_o2t.at<double>(1, 2));
		tz = tarCenterP.z - (oriCenterP.x * r_o2t.at<double>(2, 0) + oriCenterP.y * r_o2t.at<double>(2, 1) + oriCenterP.z * r_o2t.at<double>(2, 2));
		double datT[] = { tx,ty,tz };
		t_o2t = cv::Mat(3, 1, CV_64F, datT);
	
	//	std::cout << "tx:" << tx << " ty:" << ty << " tz:" << tz << std::endl;

	//	std::cout << "r:" << r_o2t << std::endl;
	//	std::cout << "t:" << t_o2t << std::endl;
		bl::R_T2H(r_o2t, t_o2t, h_o2t);
//		std::cout << "h_o2t      :" << h_o2t << std::endl;
		h_o2t = h_o2t.clone();
	

		cv::Mat htemp =  h_o2t * H_final;
//		std::cout << "htemp:\n" << htemp << std::endl;

		H_final = htemp.clone();

	//	std::cout << " count: " << iters << " err: " << err<<std::endl;


	}
//	std::cout << "H:" << H_final << std::endl;
	H2R_T(H_final, R, T);

}

void calErr(cv::Mat h, bl::vP3D& ori, bl::vP3D tar, double& err)
{

//	std::cout << "calErr: h:\n" << h << std::endl;

	int pSize = ori.size();
	double lengs = 0.0f;
	for (int i = 0; i < pSize; i++)
	{
//		std::cout <<"ori p b:" << ori[i] << std::endl;
//		std::cout <<"tar " << tar[i] << std::endl;
		cv::Mat tmp = (cv::Mat_<double>(4, 1) << ori[i].x, ori[i].y, ori[i].z, 1.0);
		cv::Mat tmp2 = h * tmp;
		ori[i].x = tmp2.at<double>(0, 0);
		ori[i].y = tmp2.at<double>(1, 0);
		ori[i].z = tmp2.at<double>(2, 0);
		
//		std::cout <<"ori p a:" << ori[i] << std::endl;

		lengs += bl::get2PointD(ori[i], tar[i]);
	}
	lengs /= pSize;
	err = lengs;

}