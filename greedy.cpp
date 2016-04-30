#include "greedy.h"

GreedyPlanner::GreedyPlanner(Vehicle x, Filter *f, int n)
{
	this->x = x;
	this->filter = f;

	this->n = n;

	/* create vector of possible actions */
	double deg = 0.0;
	double step = 2.0 * M_PI / n;
	vector<vector<float> > acts (n+1);
	vector<float> temp (2);
	int i;
	for (i = 0; i < n; i++)
	{
		acts[i] = vector<float> (2);
		acts[i][0] = 2.0 * sin(deg);
		acts[i][1] = 2.0 * cos(deg);
		deg += step;
	}
	acts[n] = vector<float> (2);
	acts[n][0] = 0.0;
	acts[n][1] = 0.0;
	this->actions = acts;
}

vector<float> GreedyPlanner::action()
{
	/* determine what the observation is */
	double o;
	if (x.sensor->type() == 0)
		o = _bearing_max;
	else
		o = 45.0; //TODO: change this to the correct field from planner

	/* update the belief */
	filter->update(x, o);

	/* loop through all possible actions, selecting best */
	vector<float> best_a = find_best_action(x);

	return best_a;
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
