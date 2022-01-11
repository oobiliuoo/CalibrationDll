#include "pch.h"
#include "BLCalibration.h"

void bl::cameraCalibration(std::string img_path_file_name, cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{

	std::cout << "=============相机标定================\n";
	std::ifstream fin(img_path_file_name); /* 标定所用图像文件的路径 */
	//读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化	
	std::cout << "开始提取角点………………\n";
	int image_count = 0;  /* 图像数量 */
	cv::Size image_size;  /* 图像的尺寸 */
	cv::Size board_size = cv::Size(11, 8);    /* 标定板上每行、列的角点数 */
	std::vector<cv::Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
	std::vector<std::vector<cv::Point2f>> image_points_seq; /* 保存检测到的所有角点 */
	std::string filename;
	std::wcout << "image_count: ";
	while (getline(fin, filename))
	{
		cv::Mat imageInput = cv::imread(filename);

		if (image_count == 1)  //读入第一张图片时获取图像宽高信息
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
		}

		/* 提取角点 */
		if (0 == cv::findChessboardCornersSB(imageInput, board_size, image_points_buf, cv::CALIB_CB_EXHAUSTIVE|cv::CALIB_CB_ACCURACY))
		{
			std::cout << "(nfc:"<<image_count<<") "; //找不到角点
			goto loop;
		}
		//else
		//{
		//	cv::Mat view_gray;
		//	cvtColor(imageInput, view_gray, CV_RGB2GRAY);
		//	/* 亚像素精确化 */
		//	cv::find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(11, 11)); //对粗提取的角点进行精确化
		//	image_points_seq.push_back(image_points_buf);  //保存亚像素角点

		//}
		image_points_seq.push_back(image_points_buf);  //保存亚像素角点
		std::cout << image_count << "  ";
		image_count++;
	loop:
		std::cout << "";

	}

	//以下是摄像机标定
	std::cout << "\n开始标定----";
	/*棋盘三维信息*/
	cv::Size square_size = cv::Size(5, 5);  /* 实际测量得到的标定板上每个棋盘格的大小 */
	std::vector<std::vector<cv::Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
	/*内外参数*/
	cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 摄像机内参数矩阵 */
	distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	std::vector<int> point_counts;  // 每幅图像中角点的数量
	std::vector<cv::Mat> tvecsMat;  /* 每幅图像的旋转向量 */
	std::vector<cv::Mat> rvecsMat; /* 每幅图像的平移向量 */
	/* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	for (t = 0; t < image_count; t++)
	{
		std::vector<cv::Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
			for (j = 0; j < board_size.width; j++)
			{
				cv::Point3f realPoint;
				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	/* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
	for (i = 0; i < image_count; i++)
	{
		point_counts.push_back(board_size.width * board_size.height);
	}


	/* 开始标定 */
	double errrr = calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);

	std::cout << "---->标定完成！\n平均重投影误差: " << errrr << std::endl;
	std::cout << "相机内参：\n" << cameraMatrix << std::endl;
	std::cout << "畸变参数：\n" << distCoeffs << std::endl;
	//对标定结果进行评价
	double total_err = 0.0; /* 所有图像的平均误差的总和 */
	double err = 0.0; /* 每幅图像的平均误差 */
	std::vector<cv::Point2f> image_points2; /* 保存重新计算得到的投影点 */
	for (i = 0; i < image_count; i++)
	{
		std::vector<cv::Point3f> tempPointSet = object_points[i];
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
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
	//	std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << std::endl;
		//	fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	std::cout << "总体平均误差：" << total_err / image_count << "像素" << std::endl;

	fin.close();

	cameraMatrix.convertTo(cameraMatrix, CV_32FC1);
	distCoeffs.convertTo(distCoeffs, CV_32FC1);

	std::cout << "=============标定完成================\n";

}

