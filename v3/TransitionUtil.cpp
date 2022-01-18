#include "TransitionUtil.h"

void TransitionUtil::H2R_T(cv::Mat H, cv::Mat& R, cv::Mat& T)
{
	cv::Rect R_rect(0, 0, 3, 3);
	cv::Rect T_rect(3, 0, 1, 3);
	R = H(R_rect);
	T = H(T_rect);
}


void TransitionUtil::R_T2H(cv::Mat R, cv::Mat T, cv::Mat& H)
{
	// CV_64F
	if (R.type() == 6) {
		cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) <<
			R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
			0, 0, 0);
		cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) <<
			T.at<double>(0, 0),
			T.at<double>(1, 0),
			T.at<double>(2, 0),
			1);
		cv::hconcat(R1, T1, H);		//矩阵拼接
	}
	else
	{
		cv::Mat_<float> R1 = (cv::Mat_<float>(4, 3) <<
			R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
			R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
			R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2),
			0, 0, 0);
		cv::Mat_<float> T1 = (cv::Mat_<float>(4, 1) <<
			T.at<float>(0, 0),
			T.at<float>(1, 0),
			T.at<float>(2, 0),
			1);

		cv::hconcat(R1, T1, H);		//矩阵拼接
	}


}

void TransitionUtil::to_CV_32F(cv::Mat& src)
{
	if (src.empty()) {
		return;
	}
	if (src.type() != 5)
		src.convertTo(src, CV_32F);

	return;

}


/**************************************************
* @brief	检查是否是旋转矩阵
* @note
* @param
* @param
* @param
* @return  true : 是旋转矩阵， false : 不是旋转矩阵
**************************************************/
bool TransitionUtil::isRotatedMatrix(cv::Mat& R)		//旋转矩阵的转置矩阵是它的逆矩阵，逆矩阵 * 矩阵 = 单位矩阵
{
	cv::Mat temp33 = R({ 0,0,3,3 });	//无论输入是几阶矩阵，均提取它的三阶矩阵
	cv::Mat Rt;
	transpose(temp33, Rt);  //转置矩阵
	cv::Mat shouldBeIdentity = Rt * temp33;//是旋转矩阵则乘积为单位矩阵
	cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

	return cv::norm(I, shouldBeIdentity) < 1e-6;
}

/**************************************************
* @brief   欧拉角转换为旋转矩阵
* @note
* @param    const std::string& seq  指定欧拉角的排列顺序；（机械臂的位姿类型有xyz,zyx,zyz几种，需要区分）
* @param    const Mat& eulerAngle   欧拉角（1*3矩阵）, 角度值
* @param
* @return   返回3*3旋转矩阵
**************************************************/
cv::Mat TransitionUtil::eulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const std::string& seq)
{
	CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);//检查参数是否正确

	eulerAngle /= (180 / CV_PI);		//度转弧度

	cv::Matx13d m(eulerAngle);				//<double, 1, 3>

	auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
	auto rxs = sin(rx), rxc = cos(rx);
	auto rys = sin(ry), ryc = cos(ry);
	auto rzs = sin(rz), rzc = cos(rz);

	//XYZ方向的旋转矩阵
	cv::Mat RotX = (cv::Mat_<double>(3, 3) << 1, 0, 0,
		0, rxc, -rxs,
		0, rxs, rxc);
	cv::Mat RotY = (cv::Mat_<double>(3, 3) << ryc, 0, rys,
		0, 1, 0,
		-rys, 0, ryc);
	cv::Mat RotZ = (cv::Mat_<double>(3, 3) << rzc, -rzs, 0,
		rzs, rzc, 0,
		0, 0, 1);
	//按顺序合成后的旋转矩阵
	cv::Mat rotMat;

	if (seq == "zyx") rotMat = RotX * RotY * RotZ;
	else if (seq == "yzx") rotMat = RotX * RotZ * RotY;
	else if (seq == "zxy") rotMat = RotY * RotX * RotZ;
	else if (seq == "yxz") rotMat = RotZ * RotX * RotY;
	else if (seq == "xyz") rotMat = RotZ * RotY * RotX;
	else if (seq == "xzy") rotMat = RotY * RotZ * RotX;
	else
	{
		std::cout << "Euler Angle Sequence string is wrong...";
	}
	if (!isRotatedMatrix(rotMat))		//欧拉角特殊情况下会出现死锁
	{
		std::cout << "Euler Angle convert to RotatedMatrix failed..." << std::endl;
		exit(-1);
	}
	return rotMat;
}



/**************************************************
* @brief   将四元数转换为旋转矩阵
* @note
* @param   const Vec4d& q   归一化的四元数: q = q0 + q1 * i + q2 * j + q3 * k;
* @return  返回3*3旋转矩阵R
**************************************************/
cv::Mat TransitionUtil::quaternionToRotatedMatrix(const cv::Vec4d& q)
{
	double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

	double q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
	double q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
	double q1q2 = q1 * q2, q1q3 = q1 * q3;
	double q2q3 = q2 * q3;
	//根据公式得来
	cv::Mat RotMtr = (cv::Mat_<double>(3, 3) << (q0q0 + q1q1 - q2q2 - q3q3), 2 * (q1q2 + q0q3), 2 * (q1q3 - q0q2),
		2 * (q1q2 - q0q3), (q0q0 - q1q1 + q2q2 - q3q3), 2 * (q2q3 + q0q1),
		2 * (q1q3 + q0q2), 2 * (q2q3 - q0q1), (q0q0 - q1q1 - q2q2 + q3q3));
	//这种形式等价
	/*Mat RotMtr = (Mat_<double>(3, 3) << (1 - 2 * (q2q2 + q3q3)), 2 * (q1q2 - q0q3), 2 * (q1q3 + q0q2),
										 2 * (q1q2 + q0q3), 1 - 2 * (q1q1 + q3q3), 2 * (q2q3 - q0q1),
										 2 * (q1q3 - q0q2), 2 * (q2q3 + q0q1), (1 - 2 * (q1q1 + q2q2)));*/

	return RotMtr;
}


/**************************************************
* @brief      将采集的原始数据转换为齐次矩阵（从机器人控制器中获得的）
* @note
* @param	  Mat& m    1*6//1*10矩阵 ， 元素为： x,y,z,rx,ry,rz  or x,y,z, q0,q1,q2,q3,rx,ry,rz
* @param	  bool useQuaternion      原始数据是否使用四元数表示
* @param	  string& seq         原始数据使用欧拉角表示时，坐标系的旋转顺序
* @return	  返回转换完的齐次矩阵
**************************************************/
cv::Mat TransitionUtil::attitudeVectorToMatrix(const cv::Mat& m, bool useQuaternion, const std::string& seq)
{
	CV_Assert(m.total() == 6 || m.total() == 10);
	//if (m.cols == 1)	//转置矩阵为行矩阵
	//	m = m.t();	

	cv::Mat temp = cv::Mat::eye(4, 4, CV_64FC1);

	if (useQuaternion)
	{
		cv::Vec4d quaternionVec = m({ 3,0,4,1 });   //读取存储的四元数
		quaternionToRotatedMatrix(quaternionVec).copyTo(temp({ 0,0,3,3 }));
	}
	else
	{
		cv::Mat rotVec;
		if (m.total() == 6)
		{
			rotVec = m({ 3,0,3,1 });   //读取存储的欧拉角
		}
		if (m.total() == 10)
		{
			rotVec = m({ 7,0,3,1 });
		}
		//如果seq为空，表示传入的是3*1旋转向量，否则，传入的是欧拉角
		if (0 == seq.compare(""))
		{
			Rodrigues(rotVec, temp({ 0,0,3,3 }));   //罗德利斯转换
		}
		else
		{
			eulerAngleToRotateMatrix(rotVec, seq).copyTo(temp({ 0,0,3,3 }));
		}
	}
	//存入平移矩阵
	temp({ 3,0,1,3 }) = m({ 0,0,3,1 }).t();
	return temp;   //返回转换结束的齐次矩阵
}
