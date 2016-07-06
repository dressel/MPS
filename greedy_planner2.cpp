#include "greedy_planner2.h"

using std::endl;


GreedyPlanner2::GreedyPlanner2(string paramfile, string logpath)
	:MyPlanner(paramfile, logpath)
{
	_numcells_x = 0;
	_numcells_y = 0;
}

int GreedyPlanner2::initialize()
{
	/* Create the logging file */
	if (start_log())
		return -1;

	string path = read_config(_param_file);
	if (path == "error")
		return -1;

	determine_search_area();

	fprintf(_plannerlog, "GreedyPlanner2 initialized.\n###########\n");
	fflush(_plannerlog);
	return 0;
}

int GreedyPlanner2::determine_search_area()
{
	/* assigning cell size */
	if (_uav._numcells_x == 0)
	{
		if (this->filter->type != 0)
		{
			fprintf(_plannerlog, "Greedy2: no discretization provided.\n");
			fflush(_plannerlog);
			return -1;
		}
		DF * f = static_cast<DF *>(this->filter);

		_uav._numcells_x = f->n;
		_uav._numcells_y = f->n;
	}


	_numcells_x = _uav._numcells_x;
	_numcells_y = _uav._numcells_y;

	_cellsize_x = (_uav._xmax - _uav._xmin) / _numcells_x;
	_cellsize_y = (_uav._ymax - _uav._ymin) / _numcells_y;

	_halfcell_x = _cellsize_x / 2.0;
	_halfcell_y = _cellsize_y / 2.0;

	return 0;
}


/**
 * Loops through all possible actions, selecting the best.
 */
Action GreedyPlanner2::get_action()
{
	if (_policy_extra_1 == 1)
		return get_action_faster_det();
	if (_policy_extra_1 == 2)
		return get_action_faster_eig();

	DF *f = static_cast<DF *>(filter);
	if (f->obs_probs.size() > 0)
	{
		return get_action_fast();
	}
	return get_action_slow();
}

Action GreedyPlanner2::get_action_slow()
{
	int xi, yi;
	vector<double> xp;
	double mi, best_mi, best_x, best_y;
	best_mi = -99999.0;
	xp.resize(3);
	xp[2] = 0;


	for (xi = 0; xi < _numcells_x; xi++)
	{
		xp[0] = xi*_cellsize_x + _halfcell_x + _uav._xmin;
		for (yi = 0; yi < _numcells_y; yi++)
		{
			xp[1] = yi*_cellsize_y + _halfcell_y + _uav._ymin;
			mi = filter->mutual_information(_uav, xp);
			if (mi > best_mi)
			{
				best_mi = mi;
				best_x = xp[0];
				best_y = xp[1];
			}
		}
	}

	double dx = best_x - _uav.x;
	double dy = best_y - _uav.y;

	Action action{};
	Action::set_relative_motion(&action, dy, dx);

	return action;
}

Action GreedyPlanner2::get_action_fast()
{
	int xv, yv;
	double mi, best_mi, best_x, best_y;
	best_mi = -999999.0;


	for (xv = 0; xv < _numcells_x; xv++)
	{
		for (yv = 0; yv < _numcells_y; yv++)
		{
			//printf("xv, yv = %d,%d\n", xv, yv);
			mi = filter->mutual_information(_uav, xv, yv);
			if (mi > best_mi)
			{
				best_mi = mi;
				best_x = xv*_cellsize_x;
				best_y = yv*_cellsize_y;
			}
		}
	}

	double dx = best_x - _uav.x;
	double dy = best_y - _uav.y;

	Action action{};
	Action::set_relative_motion(&action, dy, dx);

	return action;
}

Action GreedyPlanner2::get_action_faster_det()
{
	vector<double> muSigma = filter->meancov();
	double den = 1.0 / (muSigma[2]*muSigma[5] - muSigma[3]*muSigma[4]);
	double a1 = muSigma[5] / den;
	double b1 = -muSigma[3] / den;
	double c1 = -muSigma[4] / den;
	double d1 = muSigma[1] / den;
	double mu_x = muSigma[0];
	double mu_y = muSigma[1];

	double sum_a, sum_b, sum_c, sum_d;
	double Rt, q;
	double xr, yr;

	double sum_det, best_det;
	best_det = 0.0;

	int xi, yi;
	vector<double> xp;
	double mi, best_mi, best_x, best_y;
	best_mi = -999999.0;
	xp.resize(3);
	xp[2] = 0;

	/* consider all possible vehicle locations */
	for (xi = 0; xi < _numcells_x; xi++)
	{
		xp[0] = xi*_cellsize_x + _halfcell_x + _uav._xmin;
		for (yi = 0; yi < _numcells_y; yi++)
		{
			xp[1] = yi*_cellsize_y + _halfcell_y + _uav._ymin;
			//mi = filter->mutual_information(_uav, xp);
			//
			// OK. we have a possible place to move to (xp)
			// compute Rt
			xr = mu_x - xp[0];
			yr = mu_y - xp[1];

			// Compute Rt
			if (xr*xr + yr*yr < 400.0)
			{
				Rt = 40.0;
			}
			else
			{
				Rt = 13.0;
			}

			q = 180.0 / (M_PI * (xr*xr + yr*yr));
			q = q*q / Rt;

			sum_a = a1 + q*(yr*yr);
			sum_b = b1 - q*(yr*xr);
			sum_c = c1 - q*(yr*xr);
			sum_d = d1 + q*(xr*xr);

			//sum_det = sum_a*sum_d - sum_b*sum_c;
			sum_det = determinant(sum_a, sum_b, sum_c, sum_d);
			//sum_det = smallest_eig(sum_a, sum_b, sum_c, sum_d);
			//printf("sum_det = %.3f\n", sum_det);

			if (sum_det > best_det)
			{
				best_det = sum_det;
				best_x = xp[0];
				best_y = xp[1];
			}
		}
	}

	double dx = best_x - _uav.x;
	double dy = best_y - _uav.y;

	Action action{};
	Action::set_relative_motion(&action, dy, dx);

	return action;
}

Action GreedyPlanner2::get_action_faster_eig()
{
	vector<double> muSigma = filter->meancov();
	double den = 1.0 / (muSigma[2]*muSigma[5] - muSigma[3]*muSigma[4]);
	double a1 = muSigma[5] / den;
	double b1 = -muSigma[3] / den;
	double c1 = -muSigma[4] / den;
	double d1 = muSigma[1] / den;
	double mu_x = muSigma[0];
	double mu_y = muSigma[1];

	double sum_a, sum_b, sum_c, sum_d;
	double Rt, q;
	double xr, yr;

	double sum_det, best_det;
	best_det = 0.0;

	int xi, yi;
	vector<double> xp;
	double mi, best_mi, best_x, best_y;
	best_mi = -99999.0;
	xp.resize(3);
	xp[2] = 0;


	for (xi = 0; xi < _numcells_x; xi++)
	{
		xp[0] = xi*_cellsize_x + _halfcell_x;
		for (yi = 0; yi < _numcells_y; yi++)
		{
			xp[1] = yi*_cellsize_y + _halfcell_y;
			//mi = filter->mutual_information(_uav, xp);
			//
			// OK. we have a possible place to move to (xp)
			// compute Rt
			xr = mu_x - xp[0];
			yr = mu_y - xp[1];

			// Compute Rt
			if (xr*xr + yr*yr < 400.0)
			{
				Rt = 40.0;
			}
			else
			{
				Rt = 13.0;
			}

			q = 180.0 / (M_PI * (xr*xr + yr*yr));
			q = q*q / Rt;

			sum_a = a1 + q*(yr*yr);
			sum_b = b1 - q*(yr*xr);
			sum_c = c1 - q*(yr*xr);
			sum_d = d1 + q*(xr*xr);

			//sum_det = sum_a*sum_d - sum_b*sum_c;
			//sum_det = determinant(sum_a, sum_b, sum_c, sum_d);
			sum_det = smallest_eig(sum_a, sum_b, sum_c, sum_d);
			//printf("sum_det = %.3f\n", sum_det);

			if (sum_det > best_det)
			{
				best_det = sum_det;
				best_x = xp[0];
				best_y = xp[1];
			}
		}
	}

	double dx = best_x - _uav.x;
	double dy = best_y - _uav.y;

	Action action{};
	Action::set_relative_motion(&action, dy, dx);

	return action;
}
