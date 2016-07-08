#include "greedy_planner2.h"

using std::endl;


GreedyPlanner2::GreedyPlanner2(string paramfile, string logpath)
	:MyPlanner(paramfile, logpath)
{
	_numcells_x = 0;
	_numcells_y = 0;
	non_obs_index = 0;
}

int GreedyPlanner2::initialize()
{
	/* create the logging file */
	if (start_log()) return -1;

	/* read the configuration file */
	string path = read_config(_param_file);
	if (path == "error") return -1;

	/* additional setup for GreedyPlanner2 */
	if (determine_search_area()) return -1;

	/* should we do non-obs actions */
	if (_policy_extra_0 == 1)
	{
		string non_obs_file = path + "non_obs.csv";
		if (initialize_non_obs_actions(non_obs_file)) return -1;
	}

	/* log some stuff and exit without error */
	fprintf(_plannerlog, "GreedyPlanner2 initialized.\n###########\n");
	fflush(_plannerlog);
	return 0;
}

int GreedyPlanner2::initialize_non_obs_actions(string non_obs_path)
{
	//vector<vector<double> > non_obs_actions;
	ifstream action_stream;
	string line;
	action_stream.open(non_obs_path);
	if (!action_stream.is_open())
	{
		fprintf(_plannerlog, "Failed to find non obs file.\n");
		fflush(_plannerlog);
		return -1;
	}
	string sub;
	int current_size = 0;
	while( !getline(action_stream, line).eof() )
	{
		stringstream ss(line);

		// file shall be written in north, east, yaw commands
		non_obs_actions.resize(current_size + 1);
		non_obs_actions[current_size].resize(3);
		getline(ss, sub, ',');
		non_obs_actions[current_size][0] = stod(sub);
		getline(ss, sub, ',');
		non_obs_actions[current_size][1] = stod(sub);
		getline(ss, sub, ',');
		non_obs_actions[current_size][2] = stod(sub);

		current_size += 1;
	}

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
	{
		double (* eval_func) (double,double,double,double) = determinant;
		return get_action_faster(eval_func);
	}
	if (_policy_extra_1 == 2)
	{
		double (* eval_func) (double,double,double,double) = smallest_eig;
		return get_action_faster(eval_func);
	}

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
				best_x = xv*_cellsize_x + _halfcell_x;
				best_y = yv*_cellsize_y + _halfcell_y;
			}
		}
	}

	double dx = best_x - _uav.x;
	double dy = best_y - _uav.y;

	Action action{};
	Action::set_relative_motion(&action, dy, dx);

	return action;
}


Action GreedyPlanner2::get_action_faster( double (*eval_func)(double,double,double,double) )
{
	vector<double> muSigma = filter->meancov();
	double den = (muSigma[2]*muSigma[5] - muSigma[3]*muSigma[4]);
	double a1 = muSigma[5] / den;
	double b1 = -muSigma[3] / den;
	double c1 = -muSigma[4] / den;
	double d1 = muSigma[2] / den;
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

	BearingOnly * bo = static_cast<BearingOnly *>(_uav.sensor);
	double Rt_small = bo->noise_sigma;
	double Rt_big = 50.0;

	for (xi = 0; xi < _numcells_x; xi++)
	{
		xp[0] = xi*_cellsize_x + _halfcell_x + _uav._xmin;
		for (yi = 0; yi < _numcells_y; yi++)
		{
			xp[1] = yi*_cellsize_y + _halfcell_y + _uav._ymin;

			// OK. we have a possible place to move to (xp)
			// compute Rt
			xr = mu_x - xp[0];
			yr = mu_y - xp[1];
			
			if (xr == 0.0) xr = 0.001;
			if (yr == 0.0) yr = 0.001;

			// We have more noise if we are within 40 meters of it
			Rt = (xr*xr + yr*yr < 1600.0) ? Rt_big : Rt_small;

			q = 180.0 / (M_PI * (xr*xr + yr*yr));
			q = q*q / Rt;

			sum_a = a1 + q*(yr*yr);
			sum_b = b1 - q*(yr*xr);
			sum_c = c1 - q*(yr*xr);
			sum_d = d1 + q*(xr*xr);

			sum_det = eval_func(sum_a, sum_b, sum_c, sum_d);
			//printf("sum_det = %.3f\n", sum_det);

			if (sum_det > best_det)
			{
				//printf("sum_det = %.3f\n", sum_det);
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

Action GreedyPlanner2::get_action_no_obs() 
{
	if (_policy_extra_0)
	{
		//set index and stuff
		int num_actions = non_obs_actions.size();
		double dnorth = non_obs_actions[non_obs_index][0] - _uav.y;
		double deast = non_obs_actions[non_obs_index][1] - _uav.x;
		//double dyaw = non_obs_actions[non_obs_index][2] - _uav.heading;
		double dyaw = 0.0;

		// update the index
		non_obs_index++;
		if (non_obs_index == num_actions) non_obs_index = 0;

		/* return the action */
		Action action{};
		Action::set_relative_motion(&action, dnorth, deast);
		return action;
	}
	return get_action();
}
