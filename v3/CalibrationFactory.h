#pragma once
#include "AbsFactory.h"
#include "CamCalibration.h"
#include "ToolCalibration.h"

class CLASS_DECLSPEC CalibrationFactory : public bl::AbsCaliFactory
{
public:
	bl::AbsCamCalibration* getCamCal();
	bl::AbsToolCalibration* getToolCal();

	~CalibrationFactory() {};
};

