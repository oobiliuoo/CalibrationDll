#pragma once
#include "AbsCalibration.h"
#include "AbsUtils.h"

namespace bl {


	class AbsFactory {
	public:
		/*
			�ͷŶ���
		*/
		void deleteObject(bl::AbsCalibration* cal)
		{
			if (cal)
				delete cal;
		}

		/*
			�ͷŶ���
		*/
		void deleteObject(bl::AbsTransUtils* util)
		{
			if (util)
				delete util;
		}

		/*
			�رչ���
		*/
		virtual void close() { delete this; };

		virtual ~AbsFactory() {};
	};


	class CLASS_DECLSPEC AbsCaliFactory : public AbsFactory
	{
	public:
		/*
			��ȡ����궨����
		*/
		virtual	AbsCamCalibration* getCamCal() = 0;
	
		/*
			��ȡ���ߵı궨����
		*/
		virtual	AbsToolCalibration* getToolCal() = 0;

	
	};


	class CLASS_DECLSPEC AbsUtilsFactory : public AbsFactory
	{
	public:
		/*
			��ȡ���߶���
		*/
		virtual AbsTransUtils* getTransUtils() = 0;

	};

}

