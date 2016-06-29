#include "greedy_planner2.h"

using std::endl;


GreedyPlanner2::GreedyPlanner2(string paramfile, string logpath)
	:MyPlanner(paramfile, logpath) {}

int GreedyPlanner2::initialize()
{
	/* Create the logging file */
	if (start_log())
		return -1;

	string path = read_config(_param_file);
	if (path == "error")
		return -1;

	/* check that the filter is of  the correct type */
	if (this->filter->type != 0)
	{
		//planner_log << "Greedy2 expects a discrete filter." << endl;
		fprintf(_plannerlog, "Greedy2 expects a discrete filter.\n");
		fflush(_plannerlog);
		return -1;
	}
	DF * f = static_cast<DF *>(this->filter);
	_n = f->n;
	_cell_size = f->cell_size;

	fprintf(_plannerlog, "GreedyPlanner2 initialized.\n");
	fflush(_plannerlog);
	return 0;
}


/**
 * Loops through all possible actions, selecting the best.
 */
Action GreedyPlanner2::get_action()
{
	int xi, yi;
	vector<double> xp;
	double mi, best_mi, best_x, best_y, half_cell;
	best_mi = -99999.0;
	half_cell = _cell_size / 2;
	xp.resize(3);
	xp[2] = 0;


	for (xi = 0; xi < _n; xi++)
	{
		xp[0] = xi * _cell_size + half_cell;
		for (yi = 0; yi < _n; yi++)
		{
			xp[1] = yi * _cell_size + half_cell;
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
	vector<float> command;
	command.resize(3);
	command[0] = dy;
	command[1] = dx;
	command[2] = 0;

	Action action{};
	Action::set_relative_motion(&action, command[0], command[1]);
	//Action::set_relative_motion(&action, 10., 10.);

	//Action action{};
	//Action::set_relative_motion(&action, actions[best_i][0], actions[best_i][1]);

	return action;
}
