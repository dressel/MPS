#include "greedy_planner.h"

//GreedyPlanner::GreedyPlanner(Vehicle x, Filter *f, int n)
GreedyPlanner::GreedyPlanner(string paramfile)
{
	read_config(paramfile);

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
		acts[i][0] = x.max_step * cos(deg);  // y-direction
		acts[i][1] = x.max_step * sin(deg);  // x-direction
		acts[i][2] = 0.0;
		deg += step;
	}
	acts[n] = vector<float> (3);
	acts[n][0] = 0.0;
	acts[n][1] = 0.0;
	acts[n][2] = 0.0;
	this->actions = acts;
}

vector<float> GreedyPlanner::action()
{
	/* determine observation and update belief */
	double o = get_obs();
	filter->update(x, o);

	/* loop through all possible actions, selecting best */
	return find_best_action(x);
}


/**
 * Loops through all possible actions, selecting the best.
 */
vector<float> GreedyPlanner::find_best_action(Vehicle x)
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
		xp = x.new_pose(a);
		mi = filter->mutual_information(x, xp);
		//std::cout << "mi = " << mi << std::endl;
		if (mi > best_mi)
		{
			best_mi = mi;
			best_a = a;
		}
	}
	return best_a;
}
