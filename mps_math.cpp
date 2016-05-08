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
