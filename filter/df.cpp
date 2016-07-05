#include "df.h"

// constructor
DF::DF(double l, int n)
{
	this->type = 0;
	this->l = l;
	this->n = n;
	this->cell_size = l/n;
	this->num_bins = 36;
	this->bin_offset = 0;

	// Create the belief, then 
	this->b.resize(n);
	int i, j;
	double prob = 1.0 / (n * n);
	for (i = 0; i < n; i++)
	{
		b[i].resize(n);
		for (j = 0; j < n; j++)
		{
			b[i][j] = prob;
		}
	}
}

DF::~DF()
{
}

void DF::reset()
{
	int i, j;
	double prob = 1.0 / (n*n);
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			b[i][j] = prob;
		}
	}
}


int DF::update(Vehicle &x, double o)
{
	int ob = this->obs2bin(o, x.sensor);
	int num_cells = this->n;
	double bp_sum = 0.0;
	int theta_x, theta_y;
	double tx, ty, temp;

	for (theta_x = 0; theta_x < num_cells; theta_x++) 
	{
		for (theta_y = 0; theta_y < num_cells; theta_y++) 
		{
			tx = theta_x*cell_size + cell_size/2.0;
			ty = theta_y*cell_size + cell_size/2.0;

			this->b[theta_x][theta_y] *= O(x.x, x.y, x.heading, tx, ty, ob, x.sensor);
			bp_sum += this->b[theta_x][theta_y];
		}
	}

	int ret_val = 0;
	if (bp_sum == 0.0)
	{
		std::cout << "BELIEF CRASH" << std::endl;
		std::cout << "ob = " << ob << std::endl;
		ret_val = 1;
	}

	//normalize
	for (theta_x = 0; theta_x < num_cells; theta_x++)
	{
		for (theta_y = 0; theta_y < num_cells; theta_y++)
		{
			this->b[theta_x][theta_y] /= bp_sum;
		}
	}
	return ret_val;
}

void DF::set_obs_probs(Sensor *s)
{
	/* first resize our vector */
	int i, j;
	int n21 = 2*n - 1;
	obs_probs.resize(n21);
	for (i = 0; i < n21; i++)
	{
		obs_probs[i].resize(n21);
		for (j = 0; j < n21; j++)
		{
			obs_probs[i][j].resize(36);		// TODO: don't hardcode
		}
	}

	int xr, yr, o;
	for (xr = -n+1; xr < n; xr++)
	{
		for (yr = -n+1; yr < n; yr++)
		{
			for (o = 0; o < 36; o++)		//TODO: don't hardcode
			{
				obs_probs[xr+n-1][yr+n-1][o] = O_start(xr, yr, o, s);
			}
		}
	}
}

void DF::print_belief()
{
	int x, y;
	for (y = n-1; y >= 0; y--)
	{
		for (x = 0; x < (n-1); x++)
		{
			printf("%.2f,", b[x][y]);
		}
		printf("%.2f\n", b[n-1][y]);
	}
}

void DF::print_belief(FILE *outfile)
{
	int x, y;
	for (y = n-1; y >= 0; y--)
	{
		for (x = 0; x < (n-1); x++)
		{
			fprintf(outfile, "%.2f,", b[x][y]);
		}
		fprintf(outfile, "%.2f\n", b[n-1][y]);
	}
}

void DF::print_belief(ofstream &os)
{
	int x, y;
	os << std::setprecision(2);
	os << std::fixed;
	for (y = n-1; y >= 0; y--)
	{
		for (x = 0; x < (n-1); x++)
		{
			//printf("%.2f,", b[x][y]);
			os << b[x][y] << ",";
		}
		os << b[n-1][y] << endl;
		//printf("%.2f\n", b[n-1][y]);
	}
	os << endl;
	//printf("\n");
}


// multiple dispatch
double DF::O(double px, double py, double ph, double tx, double ty, int obs_bin, Sensor *s)
{
	double prob = 0.0;
	if (s->type() == 0)
		return O(px, py, tx, ty, obs_bin, static_cast<BearingOnly *>(s));
	if (s->type() == 1)
		return O(px, py, ph, tx, ty, obs_bin, static_cast<DirOmni *>(s));

	return prob;
}

// for the bearing only sensor
double DF::O(double px, double py, double tx, double ty, int obs_bin, BearingOnly *bo)
{
	double ang_deg = true_bearing(px, py, tx, ty);

	pair<double, double> rbe = rel_bin_edges(ang_deg, obs_bin);
	Normal d(0.0, bo->noise);
	double prob = d.cdf(rbe.second) - d.cdf(rbe.first);

	return prob;
}
//
// for the DirOmni sensor
double DF::O(double px, double py, double ph, double tx, double ty, int obs_bin, DirOmni *dom)
{
	double rel_bearing = ph - true_bearing(px, py, tx, ty);
	int rel_int = (int)rel_bearing;
	if (rel_int < 0)
		rel_int += 360;
	if (rel_int >= 360)
		rel_int -= 360;

	if (dom->_stds[rel_int] == 0)
		cout << "rel_int = " << rel_int;
	Normal d(dom->_means[rel_int], dom->_stds[rel_int]);
	return d.cdf(obs_bin+1) - d.cdf(obs_bin);
}

double DF::O(int xr, int yr, int obs_bin)
{
	return obs_probs[xr+n-1][yr+n-1][obs_bin];
}

double DF::O(int xr, int yr, int obs_bin, Sensor *s)
{
	double prob = 0.0;
	if (s->type() == 0)
		return O(xr, yr, obs_bin, static_cast<BearingOnly *>(s));
	if (s->type() == 1)
		return 0.0;		//TODO: should really error out here
		//return O(px, py, ph, tx, ty, obs_bin, static_cast<DirOmni *>(s));

	return prob;
}

double DF::O(int xr, int yr, int obs_bin, BearingOnly *bo)
{
	/*
	double ang_deg = true_bearing(xr*cell_size, yr*cell_size);
	pair<double, double> rbe = rel_bin_edges(ang_deg, obs_bin);
	Normal d(0.0, bo->noise);
	double prob = d.cdf(rbe.second) - d.cdf(rbe.first);
	*/
	return obs_probs[xr+n-1][yr+n-1][obs_bin];
}

double DF::O_start(int xr, int yr, int obs_bin, Sensor *bo)
{
	return O_start(xr, yr, obs_bin, static_cast<BearingOnly *>(bo));
}
double DF::O_start(int xr, int yr, int obs_bin, BearingOnly *bo)
{
	double ang_deg = true_bearing(xr*cell_size, yr*cell_size);
	pair<double, double> rbe = rel_bin_edges(ang_deg, obs_bin);
	Normal d(0.0, bo->noise);
	double prob = d.cdf(rbe.second) - d.cdf(rbe.first);
	return prob;
}


/**
 * Finds the start and end points for the cdf.
 *
 * returns: pair<double,double>(start, end) 
 */
pair<double,double> DF::rel_bin_edges(double bearing, int obs_bin)
{
	double start_deg, end_deg;
	double rel_start, rel_end;
	pair<double,double> b2e = bin2deg(obs_bin);
	start_deg = b2e.first;
	end_deg = b2e.second;

	rel_start = fit_180(bearing - start_deg);
	rel_end = fit_180(bearing - end_deg);

	double temp;
	if (rel_end < rel_start)
	{
		temp = rel_start;
		rel_start = rel_end;
		rel_end = temp;
	}

	// if we straddle the wrong point
	if ( (rel_end - rel_start - .0001) > (360.0/num_bins)  )
	{
		rel_start = rel_end;
		rel_end += (360.0 / num_bins);
	}
	return pair <double, double> (rel_start, rel_end);
}


// Calculate start, end degrees of bin
pair<double,double> DF::bin2deg(int obs_bin)
{
	double full_bin = 360.0 / num_bins;
	double half_bin = full_bin / 2.0;
	double start_val, end_val;
	if (obs_bin == 0)
	{
		start_val = -half_bin;
		end_val = half_bin;
	}
	else
	{
		start_val = full_bin * obs_bin - half_bin;
		end_val = full_bin * obs_bin + half_bin;
	}
	return pair <double, double> (start_val, end_val);
}

// handles the multiple dispatch
int DF::obs2bin(double o, Sensor *s)
{

	int ob = 0;
	if (s->type() == 0)
		return obs2bin(o, static_cast<BearingOnly *>(s));
	if (s->type() == 1)
		return obs2bin(o, static_cast<DirOmni *>(s));

	return ob;
}
		

int DF::obs2bin(double o, BearingOnly *s)
{
	double full_bin = 360.0 / this->num_bins;
	double half_bin = full_bin / 2.0;

	int ob = (int)((o+half_bin) / full_bin);
	if (ob == this->num_bins)
		ob = 0;
	return ob;
}

int DF::obs2bin(double o, DirOmni *s)
{
	return (int)o;
}

double DF::p_obs(Vehicle &x, double xp, double yp, double hp, int ob)
{
	double prob = 0.0;
	int theta_x, theta_y;
	double half_cell = cell_size / 2.0;
	double xj, yj;
	for (theta_x = 0; theta_x < n; theta_x++)
	{
		for (theta_y = 0; theta_y < n; theta_y++)
		{
			xj = theta_x * cell_size + half_cell;
			yj = theta_y * cell_size + half_cell;

			prob += b[theta_x][theta_y] * O(xp,yp,hp,xj,yj, ob, x.sensor);
		}
	}
	return prob;
}

double DF::p_obs2(Vehicle &x, int xv, int yv, int ob)
{
	double prob = 0.0;
	int theta_x, theta_y;
	int xr, yr;
	for (theta_x = 0; theta_x < n; theta_x++)
	{
		xr = theta_x - xv;
		for (theta_y = 0; theta_y < n; theta_y++)
		{
			yr = theta_y - yv;
			//prob += b[theta_x][theta_y] * O(xr, yr, ob, x.sensor);
			prob += b[theta_x][theta_y] * O(xr, yr, ob);
		}
	}
	return prob;
}

/**
 * Pose needs to be (x,y,heading)
 */
double DF::mutual_information(Vehicle &uav, vector<double> np)
{
	double half_cell = cell_size / 2.0;

	double H_o = 0.0;
	double H_o_t = 0.0;
	int ob, theta_x, theta_y, obo;
	double po, pot, xj, yj;

	for (ob = 0; ob < num_bins; ob ++)
	{
		obo = ob + bin_offset;
		po = p_obs(uav, np[0], np[1], np[2], obo);
		if (po > 0.0)
			H_o -= po * log(po);

		// sum over theta
		for (theta_x = 0; theta_x < n; theta_x++)
		{
			for (theta_y = 0; theta_y < n; theta_y++)
			{
				xj = theta_x * cell_size + half_cell;
				yj = theta_y * cell_size + half_cell;

				pot = O(np[0], np[1], np[2], xj, yj, obo, uav.sensor);
				if (pot > 0.0)
					H_o_t -= pot * b[theta_x][theta_y] * log(pot);
			}
		}
	}
	return H_o - H_o_t;
}

double DF::mutual_information(Vehicle &uav, int xv, int yv)
{
	double H_o = 0.0;
	double H_o_t = 0.0;
	int ob, theta_x, theta_y, obo;
	double po, pot, xj, yj;

	for (ob = 0; ob < num_bins; ob++)
	{
		obo = ob + bin_offset;
		po = p_obs2(uav, xv, yv, obo);
		if (po > 0.0)
			H_o -= po * log(po);

		// sum over theta
		for (theta_x = 0; theta_x < n; theta_x++)
		{
			for (theta_y = 0; theta_y < n; theta_y++)
			{
				//pot = O(theta_x-xv, theta_y-yv, obo, uav.sensor);
				pot = O(theta_x-xv, theta_y-yv, obo);
				if (pot > 0.0)
					H_o_t -= pot * b[theta_x][theta_y] * log(pot);
			}
		}
	}
	return H_o - H_o_t;
}

pair<double,double> DF::centroid()
{
	double x_val = 0.0;
	double y_val = 0.0;
	int xi, yi;

	for (xi = 0; xi < n; xi++)
	{
		for (yi = 0; yi < n; yi++)
		{
			x_val += (xi+0.5) * b[xi][yi];
			y_val += (yi+0.5) * b[xi][yi];
		}
	}
	pair <double,double> ret (x_val*cell_size, y_val*cell_size);
	return ret;
}

vector<double> DF::covariance()
{
	double mu_x, mu_y;
	pair<double,double> temp = centroid();
	mu_x = temp.first;
	mu_y = temp.second;
	int xi, yi;

	double c_xx = 0.0;
	double c_xy = 0.0;
	double c_yy = 0.0;
	double x = 0.0;
	double y = 0.0;

	vector<double> cov_abcd(4);
	for (xi = 0; xi < n; xi++)
	{
		for (yi = 0; yi < n; yi++)
		{
			x = (xi+0.5) * cell_size;
			y = (yi+0.5) * cell_size;

			c_xx += b[xi][yi] * x * x;
			c_yy += b[xi][yi] * y * y;
			c_xy += b[xi][yi] * (x - mu_x) * (y - mu_y);
		}
	}
	c_xx -= (mu_x * mu_x);
	c_yy -= (mu_y * mu_y);

	cov_abcd[0] = c_xx;
	cov_abcd[1] = c_xy;
	cov_abcd[2] = c_xy;
	cov_abcd[3] = c_yy;

	return cov_abcd;
}

vector<double> DF::meancov()
{
	double mu_x = 0.0;
	double mu_y = 0.0;
	int xi, yi;

	double c_xx = 0.0;
	double c_xy = 0.0;
	double c_yy = 0.0;
	double x = 0.0;
	double y = 0.0;

	vector<double> cov_abcd(6);
	for (xi = 0; xi < n; xi++)
	{
		for (yi = 0; yi < n; yi++)
		{
			x = (xi+0.5) * cell_size;
			y = (yi+0.5) * cell_size;

			mu_x += x * b[xi][yi];
			mu_y += y * b[xi][yi];

			c_xx += b[xi][yi] * x * x;
			c_yy += b[xi][yi] * y * y;
			c_xy += b[xi][yi] * x * y;
		}
	}
	c_xx -= (mu_x * mu_x);
	c_yy -= (mu_y * mu_y);
	c_xy -= (mu_x * mu_y);

	cov_abcd[0] = mu_x;
	cov_abcd[1] = mu_y;
	cov_abcd[2] = c_xx;
	cov_abcd[3] = c_xy;
	cov_abcd[4] = c_xy;
	cov_abcd[5] = c_yy;

	return cov_abcd;
}
