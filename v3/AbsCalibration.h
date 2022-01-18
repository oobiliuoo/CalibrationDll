#pragma once
#include "pch.h"

namespace bl {
	

	class AbsCalibration {
	public:
		/*
		�궨����
		CamCalibration: 1
		ToolCalibration: 2
		*/
		virtual int mType() = 0;

		virtual ~AbsCalibration() {};

	};


	class CLASS_DECLSPEC AbsCamCalibration : public AbsCalibration
	{
	public:
		AbsCamCalibration(){};

		~AbsCamCalibration(){};

		/*
		����궨
		���룺
			img_path_file_name: ��Ƭ·���ļ���
		�����
			cameraMatrix:		����ڲ�
			distCoeffs��		����ϵ��
		ע������ͼ��·�����б궨
		*/
		virtual int monocularCalbration(std::string img_path_file_name, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
			cv::Size board_size = cv::Size(11, 8), cv::Size square_size = cv::Size(5, 5)) = 0;

		// ˫Ŀ����궨
		//	void stereoCamCalbration() {};


	};


	class CLASS_DECLSPEC AbsToolCalibration : public AbsCalibration
	{

	protected:
		/*������ڲ�*/
		cv::Mat intrinsic_L;
		/*������ڲ�*/
		cv::Mat intrinsic_R;
		/*��������������ת������*/
		cv::Mat H_left2right;
		/*����������۾���*/
		cv::Mat H_cam2obj_L;
		/*����������۾���*/
		cv::Mat H_cam2obj_R;
		/*���߶˾���*/
		cv::Mat H_tool2end;
		/*������ֵ*/
		int threshold_data[2] = {0,0};
		/*��ͼ����*/
		cv::Rect rect_L;
		/*��ͼ����*/
		cv::Rect rect_R;
		/*��ʾ����*/
		int cout_f = 0;

	public:
		AbsToolCalibration() {};

		~AbsToolCalibration() {};

		/*
			����������ڲ�
			���룺 
				src: Ŀ��ֵ��3*3��
		*/
		virtual void setIntrinsic_l(cv::Mat src) = 0;

		/*
			����������ڲ�
			���룺
				src: Ŀ��ֵ��3*3��
		*/
		virtual	void setIntrinsic_r(cv::Mat src) = 0;

		/*
			������������������ת������
			���룺
				src: Ŀ��ֵ��3*3��
		*/
		virtual void setH_left2right(cv::Mat src) = 0;

		/*
			������������������ת������
				���룺
					src: Ŀ��ֵ��3*3��
		*/
		virtual void setH_cam2obj_l(cv::Mat src) = 0;

		/*
			������ͼ����
			���룺
					rect: Ŀ������
		*/
		virtual void setRect_L(cv::Rect rect) = 0;

		/*
			������ͼ����
			���룺
					rect: Ŀ������
		*/
		virtual void setRect_R(cv::Rect rect) = 0;

		/*
			����������ֵ
			���룺
				agr1: ��ͼ��ֵ
				agr2: ��ͼ��ֵ
		*/
		virtual void setThreshold(int, int) = 0;

		/*
			������ʾģʽ
			���룺
				arg: 0 �� 1 ����� 2 ͼ��� 3 �ҵ���� 4 �����������
		*/
		virtual void setCout(int) = 0;

		/*
			��ȡ���߶�ƽ������
			return : ���߶�ƽ������
		*/
		virtual cv::Mat getT_tool2end() = 0;

		/*
			˫Ŀ���߶�����
			���룺
				imgL:��ͼ
				imgR:��ͼ
				endPos: ĩ��λ�� ��2*3��<<x,y,z,rx,ry,rz;
				H_cam2base_L: ����������۾���
				intrinsic_L: ������ڲ�
				intrinsic_R: ������ڲ�
				H_left2right: ��������������ת������
				cout_flog: �ڲ���� Ĭ�Ϲر� 10086 ��ʾ�ҵ�ͼ
				rect: ��׼��Χ
			�����
				T_tool2end: ���߶˵�ĩ�˵�ƽ������ ��1*3��<<x,y,z
			return: 0 ��������
		*/
		virtual int stereoToolRepair(cv::Mat imgL, cv::Mat imgR, cv::Mat endPos, cv::Mat H_cam2base_L,
			cv::Mat intrinsic_L, cv::Mat intrinsic_R, cv::Mat H_left2right,
			cv::Mat& T_tool2end, int cout_flog = 0, cv::Rect rect = cv::Rect(0, 0, 0, 0)) = 0;

		/*
			˫Ŀ���߶�����
			���룺
				imgL:��ͼ
				imgR:��ͼ
				endPos: ĩ��λ�� ��2*3��<<x,y,z,rx,ry,rz;
			return: 0 ��������
			ע�� �����úò���
		*/
		virtual	int stereoToolRepair(cv::Mat imgL, cv::Mat imgR, cv::Mat endPos) = 0;

		/*
			˫Ŀ���߶�����
			���룺
				point_L:��ͼ��
				point_R:��ͼ��
				endPos: ĩ��λ�� ��2*3��<<x,y,z,rx,ry,rz;
			return: 0 ��������
			ע�� �����úò���
		*/
		virtual	int stereoToolRepair(cv::Point2f point_L, cv::Point2f point_R, cv::Mat endPos) = 0;

		/*
			�ҵ�
			���룺
				src: Ŀ��ͼ
				show: �����־
		*/
		virtual cv::Point2d Cusp(cv::Mat src, int show) = 0;

		/*
			�ҵ�
			���룺
				src: Ŀ��ͼ
				arg: ��ֵ
		*/
		virtual cv::Point2d Cusp2(cv::Mat src,int arg) = 0;

		/*
			˫Ŀ���ص�ת��������
			���룺
				piexl_l:��ͼ��
				piexl_r:��ͼ��
			ע�� �������úò���
				
		*/
		virtual cv::Point3f stereoPiexl2Obj(cv::Point2f& piexl_l, cv::Point2f& piexl_r) = 0;

		/*
			�󹤾߶�ת������
			���룺
				j6pos: ������λ��
			toolPoint: ���߶˻�������
		*/
		virtual void Tool2End(cv::Mat j6Pos, cv::Point3d toolPoint) = 0;

	};

}
