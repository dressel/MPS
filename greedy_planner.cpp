#include "greedy_planner.h"

//GreedyPlanner::GreedyPlanner(Vehicle x, Filter *f, int n)
	//_param_file = paramfile;
GreedyPlanner::GreedyPlanner(string paramfile)
{
	_param_file = paramfile;
}

bool GreedyPlanner::initialize()
{
	string path = read_config(_param_file);
	if (path == "error")
	{
		// TODO: log this somewhere
		return false;
	}

	/* create vector of possible actions */
	int n = 8;
	this->n = n;
	double deg = 0.0;
	double step = 2.0 * M_PI / n;
	vector<vector<float> > acts (n+1);
	int i;
	for (i = 0; i < n; i++)
	{
		acts[i] = vector<float> (3);
		acts[i][0] = _uav.max_step * cos(deg);  // y-direction
		acts[i][1] = _uav.max_step * sin(deg);  // x-direction
		acts[i][2] = 0.0;
		deg += step;
	}
	acts[n] = vector<float> (3);
	acts[n][0] = 0.0;
	acts[n][1] = 0.0;
	acts[n][2] = 0.0;
	this->actions = acts;

	return true;
}

vector<float> GreedyPlanner::action()
{
	/* determine observation and update belief */
	double o = get_obs();
	filter->update(_uav, o);

	/* loop through all possible actions, selecting best */
	return find_best_action(_uav);
}


/**
 * Loops through all possible actions, selecting the best.
 */
vector<float> GreedyPlanner::find_best_action(Vehicle uav)
{
	int i;
	vector<float> a;
	vector<float> best_a;
	vector<double> xp;
	double mi, best_mi;
	best_mi = -100.0; //TODO: is this actually ok?
	for (i = 0; i <= n; i++)
	{
		/* find out where action takes you */
		a = actions[i];
		xp = uav.new_pose(a);
		mi = filter->mutual_information(uav, xp);
		//std::cout << "mi = " << mi << std::endl;
		if (mi > best_mi)
		{
			best_mi = mi;
			best_a = a;
		}
	}
	return best_a;
}
