#include "CalibrationFactory.h"

bl::AbsCamCalibration* CalibrationFactory::getCamCal() 
{
	return new CamCalibration();
}

bl::AbsToolCalibration* CalibrationFactory::getToolCal()
{
	return new ToolCalibration();
}


