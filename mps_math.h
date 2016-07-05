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
double true_bearing(double xr, double yr);

double fit_180(double angle);

/*
 * Matrix is of the form
 * [a b; c d]
 */
double determinant(double a, double b, double c, double d);
double smallest_eig(double a, double b, double c, double d);
