#include "greedy_planner.h"

using std::endl;


GreedyPlanner::GreedyPlanner(string paramfile, string logpath)
	:MyPlanner(paramfile, logpath) {}

int GreedyPlanner::initialize()
{
	/* Create the logging file */
	if (start_log())
		return -1;

	string path = read_config(_param_file);
	if (path == "error")
		return -1;

	/* Initialization can depend on sensor type */
	int s_type = _uav.sensor->type();
	planner_log << "s type = " << s_type;
	if (s_type == 0)
		return bo_initialize();
	if (s_type == 1)
		return do_initialize();

	/* unrecognized sensor type, this should error out */
	return -1;
}


/**
 * Loops through all possible actions, selecting the best.
 */
vector<float> GreedyPlanner::get_action()
{
	int i, best_i;
	vector<float> a;
	vector<double> xp;
	double mi, best_mi;
	best_mi = -99999.0;
	int num_actions = actions.size();

	for (i = 0; i < num_actions; i++)
	{
		xp = _uav.new_pose(actions[i]); 
		mi = filter->mutual_information(_uav, xp);
		if (mi > best_mi)
		{
			best_mi = mi;
			best_i = i;
		}
	}

	return actions[best_i];
}

int GreedyPlanner::bo_initialize()
{
	int num_angles = 8;
	double deg = 0.0;
	double step = 2.0 * M_PI / num_angles;
	int i;
	actions.resize(num_angles+1);

	for (i = 0; i < num_angles; i++)
	{
		actions[i].resize(3);
		actions[i][0] = _uav.max_step * cos(deg);  // y-direction
		actions[i][1] = _uav.max_step * sin(deg);  // x-direction
		actions[i][2] = 0.0;
		deg += step;
	}
	actions[num_angles].resize(3);
	actions[num_angles][0] = 0.0;
	actions[num_angles][1] = 0.0;
	actions[num_angles][2] = 0.0;

	return 0;
}

/* directional + omni initialize */
int GreedyPlanner::do_initialize()
{
	int num_angles = 8;
	int num_actions = 3*(num_angles + 1);
	double deg = 0.0;
	double step = 2.0 * M_PI / num_angles;
	vector<vector<float> > acts (num_actions);
	int i;
	double x_step, y_step;
	actions.resize(num_actions);
	for (i = 0; i < num_angles; i++)
	{
		y_step = _uav.max_step * cos(deg);
		x_step = _uav.max_step * sin(deg);

		actions[i].resize(3);
		actions[i][0] = y_step;  // y-direction
		actions[i][1] = x_step;  // x-direction
		actions[i][2] = 0.0;

		actions[i+num_angles].resize(3);
		actions[i+num_angles][0] = _uav.max_step * cos(deg);  // y-direction
		actions[i+num_angles][1] = _uav.max_step * sin(deg);  // x-direction
		actions[i+num_angles][2] = -10.0;

		actions[i+2*num_angles].resize(3);
		actions[i+2*num_angles][0] = y_step;  // y-direction
		actions[i+2*num_angles][1] = x_step;  // x-direction
		actions[i+2*num_angles][2] = 10.0;

		deg += step;
	}

	/* Add the actions for no movement at all */
	actions[num_actions-3].resize(3);
	actions[num_actions-3][0] = 0.0;
	actions[num_actions-3][1] = 0.0;
	actions[num_actions-3][2] = 0.0;
	actions[num_actions-2].resize(3);
	actions[num_actions-2][0] = 0.0;
	actions[num_actions-2][1] = 0.0;
	actions[num_actions-2][2] = -10.0;
	actions[num_actions-1].resize(3);
	actions[num_actions-1][0] = 0.0;
	actions[num_actions-1][1] = 0.0;
	actions[num_actions-1][2] = 10.0;

	return 0;
}
