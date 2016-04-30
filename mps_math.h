#include <math.h>

class Normal
{
	public:
		double mu;
		double sigma;
		Normal(double mu, double sigma);
		double cdf(double x);
};
