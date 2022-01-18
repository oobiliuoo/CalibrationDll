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
		cv::hconcat(R1, T1, H);		//����ƴ��
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

		cv::hconcat(R1, T1, H);		//����ƴ��
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
* @brief	����Ƿ�����ת����
* @note
* @param
* @param
* @param
* @return  true : ����ת���� false : ������ת����
**************************************************/
bool TransitionUtil::isRotatedMatrix(cv::Mat& R)		//��ת�����ת�þ������������������� * ���� = ��λ����
{
	cv::Mat temp33 = R({ 0,0,3,3 });	//���������Ǽ��׾��󣬾���ȡ�������׾���
	cv::Mat Rt;
	transpose(temp33, Rt);  //ת�þ���
	cv::Mat shouldBeIdentity = Rt * temp33;//����ת������˻�Ϊ��λ����
	cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

	return cv::norm(I, shouldBeIdentity) < 1e-6;
}

/**************************************************
* @brief   ŷ����ת��Ϊ��ת����
* @note
* @param    const std::string& seq  ָ��ŷ���ǵ�����˳�򣻣���е�۵�λ��������xyz,zyx,zyz���֣���Ҫ���֣�
* @param    const Mat& eulerAngle   ŷ���ǣ�1*3����, �Ƕ�ֵ
* @param
* @return   ����3*3��ת����
**************************************************/
cv::Mat TransitionUtil::eulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const std::string& seq)
{
	CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);//�������Ƿ���ȷ

	eulerAngle /= (180 / CV_PI);		//��ת����

	cv::Matx13d m(eulerAngle);				//<double, 1, 3>

	auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
	auto rxs = sin(rx), rxc = cos(rx);
	auto rys = sin(ry), ryc = cos(ry);
	auto rzs = sin(rz), rzc = cos(rz);

	//XYZ�������ת����
	cv::Mat RotX = (cv::Mat_<double>(3, 3) << 1, 0, 0,
		0, rxc, -rxs,
		0, rxs, rxc);
	cv::Mat RotY = (cv::Mat_<double>(3, 3) << ryc, 0, rys,
		0, 1, 0,
		-rys, 0, ryc);
	cv::Mat RotZ = (cv::Mat_<double>(3, 3) << rzc, -rzs, 0,
		rzs, rzc, 0,
		0, 0, 1);
	//��˳��ϳɺ����ת����
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
	if (!isRotatedMatrix(rotMat))		//ŷ������������»��������
	{
		std::cout << "Euler Angle convert to RotatedMatrix failed..." << std::endl;
		exit(-1);
	}
	return rotMat;
}



/**************************************************
* @brief   ����Ԫ��ת��Ϊ��ת����
* @note
* @param   const Vec4d& q   ��һ������Ԫ��: q = q0 + q1 * i + q2 * j + q3 * k;
* @return  ����3*3��ת����R
**************************************************/
cv::Mat TransitionUtil::quaternionToRotatedMatrix(const cv::Vec4d& q)
{
	double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

	double q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
	double q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
	double q1q2 = q1 * q2, q1q3 = q1 * q3;
	double q2q3 = q2 * q3;
	//���ݹ�ʽ����
	cv::Mat RotMtr = (cv::Mat_<double>(3, 3) << (q0q0 + q1q1 - q2q2 - q3q3), 2 * (q1q2 + q0q3), 2 * (q1q3 - q0q2),
		2 * (q1q2 - q0q3), (q0q0 - q1q1 + q2q2 - q3q3), 2 * (q2q3 + q0q1),
		2 * (q1q3 + q0q2), 2 * (q2q3 - q0q1), (q0q0 - q1q1 - q2q2 + q3q3));
	//������ʽ�ȼ�
	/*Mat RotMtr = (Mat_<double>(3, 3) << (1 - 2 * (q2q2 + q3q3)), 2 * (q1q2 - q0q3), 2 * (q1q3 + q0q2),
										 2 * (q1q2 + q0q3), 1 - 2 * (q1q1 + q3q3), 2 * (q2q3 - q0q1),
										 2 * (q1q3 - q0q2), 2 * (q2q3 + q0q1), (1 - 2 * (q1q1 + q2q2)));*/

	return RotMtr;
}


/**************************************************
* @brief      ���ɼ���ԭʼ����ת��Ϊ��ξ��󣨴ӻ����˿������л�õģ�
* @note
* @param	  Mat& m    1*6//1*10���� �� Ԫ��Ϊ�� x,y,z,rx,ry,rz  or x,y,z, q0,q1,q2,q3,rx,ry,rz
* @param	  bool useQuaternion      ԭʼ�����Ƿ�ʹ����Ԫ����ʾ
* @param	  string& seq         ԭʼ����ʹ��ŷ���Ǳ�ʾʱ������ϵ����ת˳��
* @return	  ����ת�������ξ���
**************************************************/
cv::Mat TransitionUtil::attitudeVectorToMatrix(const cv::Mat& m, bool useQuaternion, const std::string& seq)
{
	CV_Assert(m.total() == 6 || m.total() == 10);
	//if (m.cols == 1)	//ת�þ���Ϊ�о���
	//	m = m.t();	

	cv::Mat temp = cv::Mat::eye(4, 4, CV_64FC1);

	if (useQuaternion)
	{
		cv::Vec4d quaternionVec = m({ 3,0,4,1 });   //��ȡ�洢����Ԫ��
		quaternionToRotatedMatrix(quaternionVec).copyTo(temp({ 0,0,3,3 }));
	}
	else
	{
		cv::Mat rotVec;
		if (m.total() == 6)
		{
			rotVec = m({ 3,0,3,1 });   //��ȡ�洢��ŷ����
		}
		if (m.total() == 10)
		{
			rotVec = m({ 7,0,3,1 });
		}
		//���seqΪ�գ���ʾ�������3*1��ת���������򣬴������ŷ����
		if (0 == seq.compare(""))
		{
			Rodrigues(rotVec, temp({ 0,0,3,3 }));   //�޵���˹ת��
		}
		else
		{
			eulerAngleToRotateMatrix(rotVec, seq).copyTo(temp({ 0,0,3,3 }));
		}
	}
	//����ƽ�ƾ���
	temp({ 3,0,1,3 }) = m({ 0,0,3,1 }).t();
	return temp;   //����ת����������ξ���
}
