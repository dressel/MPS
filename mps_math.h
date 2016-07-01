#include <math.h>

class Normal
{
	public:
		double mu;
		double sigma;
		Normal(double mu, double sigma);
		double cdf(double x);
};

double true_bearing(double px, double py, double tx, double ty);
