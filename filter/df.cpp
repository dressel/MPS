#include "df.h"

// constructor
DF::DF(double l, int n)
{
	this->type = 0;
	this->l = l;
	this->n = n;
	this->cell_size = l/n;
	this->num_bins = 36;

	// Create the belief, then 
	this->b = vector<vector<double> >(n);
	int i;
	double prob = 1.0 / (n * n);
	for (i = 0; i < n; i++)
	{
		b[i] = vector<double>(n, prob);
	}
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


int DF::update(Vehicle x, double o)
{
	int ob = this->obs2bin(o, x.sensor);
	int num_cells = this->n;
	double cell_size = this->cell_size;
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
	//print_belief();
	return ret_val;
}

//TODO: is there a performance hit here?
// it doesn't matter now, but it could later
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
	printf("\n");
}


// multiple dispatch
double DF::O(double px, double py, double ph, double tx, double ty, int obs_bin, Sensor *s)
{
	double prob = 0.0;
	if (s->type() == 0)
		prob = O(px, py, tx, ty, obs_bin, static_cast<BearingOnly *>(s));
	return prob;
}

// for the bearing only sensor
double DF::O(double px, double py, double tx, double ty, int obs_bin, BearingOnly *bo)
{
	double ang_deg = true_bearing(px, py, tx, ty);

	pair<double, double> rbe = rel_bin_edges(ang_deg, obs_bin);
	//Normal d = Normal(0.0, bo->noise);
	Normal d(0.0, bo->noise);
	double prob = d.cdf(rbe.second) - d.cdf(rbe.first);


	return prob;
	// rel_start, rel_end = rel_bin_edges(ang_deg, o, df)
	// d = Normal(0, x.sensor.noise_sigma)
	// p = cdf(d, rel_end) - cdf(d, rel_start)
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

double DF::fit_180(double angle)
{
	if (angle > 180.0)
		angle -= 360.0;
	else if (angle < -180.0)
		angle += 360.0;
	return angle;

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
		ob = this->obs2bin(o, static_cast<BearingOnly *>(s));
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

double DF::true_bearing(double px, double py, double tx, double ty)
{
	double xr = tx - px;
	double yr = ty - py;
	double ang_deg = atan2(xr,yr) * 180.0 / M_PI;
	if (ang_deg < 0.0)
		ang_deg += 360.0;
	return ang_deg;
}

double DF::p_obs(Vehicle x, double xp, double yp, double hp, int ob)
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
double DF::mutual_information(Vehicle x, vector<double> np)
{
	double nh = 3.0;
	double prob = p_obs(x, np[0], np[1], nh, 0);
	double half_cell = cell_size / 2.0;

	double H_o = 0.0;
	double H_o_t = 0.0;
	int ob, theta_x, theta_y;
	double po, pot, xj, yj;

	for (ob = 0; ob < 36; ob ++)
	{
		po = p_obs(x, np[0], np[1], nh, ob);
		if (po > 0.0)
			H_o -= po * log(po);

		// sum over theta
		for (theta_x = 0; theta_x < n; theta_x++)
		{
			for (theta_y = 0; theta_y < n; theta_y++)
			{
				xj = theta_x * cell_size + half_cell;
				yj = theta_y * cell_size + half_cell;

				pot = O(np[0], np[1], 0.0, xj, yj, ob, x.sensor);
				if (pot > 0.0)
					H_o_t -= pot * b[theta_x][theta_y] * log(pot);
			}
		}
	}
	return H_o - H_o_t;
}
