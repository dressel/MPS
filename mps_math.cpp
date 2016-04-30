#include <iostream>  //temp
#include "mps_math.h"

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
		std::cout << "Normal: temp = " <<temp << ", tempans = " << tempans << std::endl;
		std::cout << "x = " << x <<std::endl;
		std::cout << "mu = " << mu <<std::endl;
		std::cout << "sigma = " << sigma <<std::endl;
	}
		
	return 0.5 * (1 + erf(temp));
}
