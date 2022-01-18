#pragma once
#include "AbsCalibration.h"
#include "AbsUtils.h"

namespace bl {


	class AbsFactory {
	public:
		/*
			释放对象
		*/
		void deleteObject(bl::AbsCalibration* cal)
		{
			if (cal)
				delete cal;
		}

		/*
			释放对象
		*/
		void deleteObject(bl::AbsTransUtils* util)
		{
			if (util)
				delete util;
		}

		/*
			关闭工厂
		*/
		virtual void close() { delete this; };

		virtual ~AbsFactory() {};
	};


	class CLASS_DECLSPEC AbsCaliFactory : public AbsFactory
	{
	public:
		/*
			获取相机标定对象
		*/
		virtual	AbsCamCalibration* getCamCal() = 0;
	
		/*
			获取工具的标定对象
		*/
		virtual	AbsToolCalibration* getToolCal() = 0;

	
	};


	class CLASS_DECLSPEC AbsUtilsFactory : public AbsFactory
	{
	public:
		/*
			获取工具对象
		*/
		virtual AbsTransUtils* getTransUtils() = 0;

	};

}

