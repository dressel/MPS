#include <iostream>  //temp
#include "mps_math.h"

using std::cout;
using std::endl;

Normal::Normal(double mu, double sigma)
{
	this->mu = mu;
	this->sigma = sigma;
}

double Normal::cdf(double x)
{
	double temp = (x - mu) / (sigma * sqrt(2.0));
	double tempans = 0.5 * (1 + erf(temp));
	if isnan((tempans))
	{
		cout << "Normal: temp = " << temp << ", tempans = " << tempans << endl;
		cout << "x = " << x << ", mu = " << mu << ", sigma = " << sigma << endl;
	}
		
	return 0.5 * (1 + erf(temp));
}


double true_bearing(double px, double py, double tx, double ty)
{
	double xr = tx - px;
	double yr = ty - py;
	double ang_deg = atan2(xr,yr) * 180.0 / M_PI;
	if (ang_deg < 0.0)
		ang_deg += 360.0;
	return ang_deg;
}
