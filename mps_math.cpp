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

double cdf(double mu, double sigma, double x)
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

/**
 * xr = x_jammer - x_vehicle
 * yr = y_jammer - y_vehicle
 */
double true_bearing(double xr, double yr)
{
	double ang_deg = atan2(xr,yr) * 180.0 / M_PI;
	if (ang_deg < 0.0)
		ang_deg += 360.0;
	return ang_deg;
}

double fit_180(double angle)
{
	if (angle > 180.0)
		angle -= 360.0;
	else if (angle < -180.0)
		angle += 360.0;
	return angle;
}

double determinant(double a, double b, double c, double d)
{
	return a*d - b*c;
}

double smallest_eig(double a, double b, double c, double d)
{
	double B = -1.0 * (d+a);
	double C = a*d - b*c;
	double a1 = (-B + sqrt(B*B - 4*C)) / 2.0;
	double a2 = (-B - sqrt(B*B - 4*C)) / 2.0;

	if (a1 < a2)
		return a1;

	return a2;
}
