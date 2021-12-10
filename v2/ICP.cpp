#include "pch.h"
#include "BLCalibration.h"


void calNearestPointPairs(cv::Mat h, CvMat* ori, CvMat* tar, bl::vP3F targetPoints, double& err);


void bl::ICP(vP3F originPoints, vP3F targetPoints, cv::Mat& R, cv::Mat& T)
{

	// 1.参数设置
	// 错误率
	double err = 1.0;
	// 最高错误率
	double maxErr = 0.2;
	// 迭代次数 
	int maxIter = 100;
	// 设定初始R、T、h
	cv::Mat r_o2t = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	cv::Mat t_o2t = (cv::Mat_<float>(3, 1) << 0, 0, 0);
	cv::Mat h_o2t;
	bl::R_T2H(r_o2t, t_o2t, h_o2t);
	cv::Mat H_final = h_o2t;

	int pointSize = originPoints.size();
	//构建源点集cvmat
	CvMat* pointsOri = cvCreateMat(pointSize, 3, CV_32FC1);
	CvMat* pointsTar = cvCreateMat(pointSize, 3, CV_32FC1);

	// 赋值
	for (int i = 0; i < pointSize; i++)
	{
		pointsOri->data.fl[i * 3 + 0] = originPoints[i].x; 
		pointsOri->data.fl[i * 3 + 1] = originPoints[i].y; 
		pointsOri->data.fl[i * 3 + 2] = originPoints[i].z; 

		pointsTar->data.fl[i * 3 + 0] = targetPoints[i].x; 
		pointsTar->data.fl[i * 3 + 1] = targetPoints[i].y; 
		pointsTar->data.fl[i * 3 + 2] = targetPoints[i].z; 

	}

	int nrows = pointsOri->rows;
	int ncols = pointsOri->cols;
	int type = pointsOri->type;

	int iters = 0;
	while (err > maxErr && iters < maxIter)
	{

		iters++;
		double lastErr = err;
		// 计算临近点对
		calNearestPointPairs(h_o2t, pointsOri,pointsTar, targetPoints, err);

		CvMat* centroidOri = cvCreateMat(1, ncols, type);
		CvMat* centroidTar = cvCreateMat(1, ncols, type);
		cvSet(centroidOri, cvScalar(0));
		cvSet(centroidTar, cvScalar(0));
		for (int c = 0; c < ncols; c++) {
			for (int r = 0; r < nrows; r++)
			{
				centroidOri->data.fl[c] += pointsOri->data.fl[ncols * r + c];
				centroidTar->data.fl[c] += pointsTar->data.fl[ncols * r + c];
			}
			centroidOri->data.fl[c] /= nrows;
			centroidTar->data.fl[c] /= nrows;
		}

		// 去质心
		for (int r = 0; r < nrows; r++)
		{
			for (int c = 0; c < ncols; c++)
			{
				pointsOri->data.fl[ncols * r + c] = pointsOri->data.fl[ncols * r + c] - centroidOri->data.fl[c];
				pointsTar->data.fl[ncols * r + c] = pointsTar->data.fl[ncols * r + c] - centroidTar->data.fl[c];
			}
		}
	
		// 奇异值分解
		CvMat* H = cvCreateMat(ncols, ncols, type);
		CvMat* W = cvCreateMat(ncols, ncols, type);
		CvMat* V = cvCreateMat(ncols, ncols, type);
		CvMat* U = cvCreateMat(ncols, ncols, type);



		/*
		
		double cvGEMM(//矩阵的广义乘法运算
			const CvArr* src1,//乘数矩阵
			const CvArr* src2,//乘数矩阵
			double alpha,//1号矩阵系数
			const CvArr* src3,//加权矩阵
			double beta,//2号矩阵系数
			CvArr* dst,//结果矩阵
			int tABC = 0//变换标记
		);
		函数对应的乘法运算公式为：dst = (alpha*src1)xsrc2+(beta*src3)
		CV_GEMM_A_T 转置 src1
		CV_GEMM_B_T 转置 src2
		CV_GEMM_C_T 转置 src3
		*/

		cvGEMM(pointsOri, pointsTar, 1, NULL, 0, H, CV_GEMM_B_T);

		/*
		void cvSVD(//计算 A = U*W*(V的转置)
			CvArr* A,
			CvArr* W,
			CvArr* U = NULL,
			CvArr* V = NULL,
			int flags = 0//标记位
		参数	结果
			CV_SVD_MODIFY_A	允许改变矩阵A
			CV_SVD_U_T	返回U转置而不是U
			CV_SVD_V_T	返回V转置而不是V
		);
		*/
		cvSVD(H, W, U, V, CV_SVD_U_T);
	
		// R =V*U.t
		cv::Mat U1(U->rows,U->cols,U->type,U->data.fl);
		cv::Mat V1(V->rows,V->cols,V->type,V->data.fl);
		double m = cv::norm(U1 * V1);
		cv::Mat c = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, m);
		r_o2t = V1 *c* U1;

		// T = cpt - R * cps
		cv::Mat cpt(centroidTar->rows,centroidTar->cols,centroidTar->type,centroidTar->data.fl);
		cv::Mat cps(centroidOri->rows,centroidOri->cols,centroidOri->type,centroidOri->data.fl);
	//	std::cout << "cpt: \n" << cpt<<std::endl;
	//	std::cout << "cps: \n" << cps<<std::endl;

		t_o2t = cpt.t() - r_o2t * cps.t();


		bl::R_T2H(r_o2t, t_o2t, h_o2t);

		H_final = h_o2t * H_final;
		//std::cout << "H:" << h_o2t << std::endl;

		std::cout << " count: " << iters << " err: " << err<<std::endl;
		// 释放资源

		cvReleaseMat(&centroidOri);
		cvReleaseMat(&centroidTar);
		cvReleaseMat(&H);
		cvReleaseMat(&U);
		cvReleaseMat(&V);


	}
	cvReleaseMat(&pointsOri);
	cvReleaseMat(&pointsTar);
	std::cout << "H:" << H_final << std::endl;
	R = r_o2t.clone();
	T = t_o2t.clone();

}

void calNearestPointPairs(cv::Mat h, CvMat* ori, CvMat* tar,bl::vP3F targetPoint, double& err)
{
	cv::Mat oriMat(ori->rows, ori->cols, ori->type, ori->data.fl);
	int pointNum = oriMat.rows;
	bl::vP3F pointsOri;
	for (int i = 0; i < pointNum; i++)
	{
		float* ptr = oriMat.ptr<float>(i);
		cv::Mat pMat = (cv::Mat_<float>(4,1)<<*(ptr), *(ptr+1), *(ptr+2),1);
		pMat = h * pMat;
		float* ptr2 = pMat.ptr<float>(0);
		cv::Point3f p(*(ptr2),*(ptr2+1),*(ptr2+2));
		pointsOri.push_back(p);
	}


	// 赋值
	for (int i = 0; i < pointNum; i++)
	{
		ori->data.fl[i * 3 + 0] = pointsOri[i].x;
		ori->data.fl[i * 3 + 1] = pointsOri[i].y;
		ori->data.fl[i * 3 + 2] = pointsOri[i].z;

		tar->data.fl[i * 3 + 0] = targetPoint[i].x;
		tar->data.fl[i * 3 + 1] = targetPoint[i].y;
		tar->data.fl[i * 3 + 2] = targetPoint[i].z;
	
	}

	// 求误差
	for (int i = 0; i < pointNum; i++)
	{
		err +=	bl::get2PointD(pointsOri[i], targetPoint[i]);
	}
	err /= pointNum;
	
//	std::cout << "err:" << err<<std::endl;



}