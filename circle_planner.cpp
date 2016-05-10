#include "circle_planner.h"

CirclePlanner::CirclePlanner(string paramfile, string logpath)
	: MyPlanner(paramfile, logpath)
{
}

int CirclePlanner::initialize()
{
	/* Create the logging file */
	if (start_log())
		return -1;

	/* read the parameter file, logging any errors */
	string path = read_config(_param_file);
	if (path == "error")
		return -1;

	// handle last action
	this->last.resize(2);
	this->last[0] = 0.0;
	this->last[1] = 0.0;
	return 0;
}

vector<float> CirclePlanner::action()
{
	update_belief();

	// ok, we've updated belief. Now pick action
	float ax = -cos(_bearing_max * M_PI/180.0);
	float ay = sin(_bearing_max * M_PI/180.0);
	
	// prevent circle from switching directions
	// if dot product is negative from last attempt, direction is different
	if (ax*last[0] + ay*last[1] < 0.0)
	{
		ax = -ax;
		ay = -ay;
	}
	vector<float>commands (3);
	commands[0] = _uav.max_step * ay;
	commands[1] = _uav.max_step * ax;
	commands[2] = 0.0;

	// preserve current action as last to check for direction
	last[0] = ax;
	last[1] = ay;

	// We keep track of vehicle's movement here
	_uav.move(ax, ay);

	return commands;
}
