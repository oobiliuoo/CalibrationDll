#include "pch.h"
#include "BLCalibration.h"


double bl::getCosines(double a,double b, double c)
{
	return (a * a + b * b - c * c) /( 2 * a * b);
}



