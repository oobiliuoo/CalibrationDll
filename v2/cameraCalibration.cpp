#include "pch.h"
#include "BLCalibration.h"

void bl::cameraCalibration(std::string img_path_file_name, cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{

	std::cout << "=============����궨================\n";
	std::ifstream fin(img_path_file_name); /* �궨����ͼ���ļ���·�� */
	//��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��	
	std::cout << "��ʼ��ȡ�ǵ㡭����������\n";
	int image_count = 0;  /* ͼ������ */
	cv::Size image_size;  /* ͼ��ĳߴ� */
	cv::Size board_size = cv::Size(11, 8);    /* �궨����ÿ�С��еĽǵ��� */
	std::vector<cv::Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	std::vector<std::vector<cv::Point2f>> image_points_seq; /* �����⵽�����нǵ� */
	std::string filename;
	std::wcout << "image_count: ";
	while (getline(fin, filename))
	{
		cv::Mat imageInput = cv::imread(filename);

		if (image_count == 1)  //�����һ��ͼƬʱ��ȡͼ������Ϣ
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
		}

		/* ��ȡ�ǵ� */
		if (0 == cv::findChessboardCornersSB(imageInput, board_size, image_points_buf, cv::CALIB_CB_EXHAUSTIVE|cv::CALIB_CB_ACCURACY))
		{
			std::cout << "(nfc:"<<image_count<<") "; //�Ҳ����ǵ�
			goto loop;
		}
		//else
		//{
		//	cv::Mat view_gray;
		//	cvtColor(imageInput, view_gray, CV_RGB2GRAY);
		//	/* �����ؾ�ȷ�� */
		//	cv::find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(11, 11)); //�Դ���ȡ�Ľǵ���о�ȷ��
		//	image_points_seq.push_back(image_points_buf);  //���������ؽǵ�

		//}
		image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
		std::cout << image_count << "  ";
		image_count++;
	loop:
		std::cout << "";

	}

	//������������궨
	std::cout << "\n��ʼ�궨----";
	/*������ά��Ϣ*/
	cv::Size square_size = cv::Size(5, 5);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
	std::vector<std::vector<cv::Point3f>> object_points; /* ����궨���Ͻǵ����ά���� */
	/*�������*/
	cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* ������ڲ������� */
	distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* �������5������ϵ����k1,k2,p1,p2,k3 */
	std::vector<int> point_counts;  // ÿ��ͼ���нǵ������
	std::vector<cv::Mat> tvecsMat;  /* ÿ��ͼ�����ת���� */
	std::vector<cv::Mat> rvecsMat; /* ÿ��ͼ���ƽ������ */
	/* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	for (t = 0; t < image_count; t++)
	{
		std::vector<cv::Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
			for (j = 0; j < board_size.width; j++)
			{
				cv::Point3f realPoint;
				/* ����궨�������������ϵ��z=0��ƽ���� */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	/* ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨�� */
	for (i = 0; i < image_count; i++)
	{
		point_counts.push_back(board_size.width * board_size.height);
	}


	/* ��ʼ�궨 */
	double errrr = calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);

	std::cout << "---->�궨��ɣ�\nƽ����ͶӰ���: " << errrr << std::endl;
	std::cout << "����ڲΣ�\n" << cameraMatrix << std::endl;
	std::cout << "���������\n" << distCoeffs << std::endl;
	//�Ա궨�����������
	double total_err = 0.0; /* ����ͼ���ƽ�������ܺ� */
	double err = 0.0; /* ÿ��ͼ���ƽ����� */
	std::vector<cv::Point2f> image_points2; /* �������¼���õ���ͶӰ�� */
	for (i = 0; i < image_count; i++)
	{
		std::vector<cv::Point3f> tempPointSet = object_points[i];
		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
		std::vector<cv::Point2f> tempImagePoint = image_points_seq[i];
		cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
		cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
		total_err += err /= point_counts[i];
	//	std::cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << std::endl;
		//	fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	}
	std::cout << "����ƽ����" << total_err / image_count << "����" << std::endl;

	fin.close();

	cameraMatrix.convertTo(cameraMatrix, CV_32FC1);
	distCoeffs.convertTo(distCoeffs, CV_32FC1);

	std::cout << "=============�궨���================\n";

}

