#include "circle_planner.h"

//CirclePlanner::CirclePlanner(Vehicle x, Filter *f)
CirclePlanner::CirclePlanner(string paramfile)
{
	read_config(paramfile);

	// handle last action
	this->last.resize(2);
	this->last[0] = 0.0;
	this->last[1] = 0.0;
}

bool CirclePlanner::initialize()
{
	return true;
}

vector<float> CirclePlanner::action()
{
	double o = get_obs();
	filter->update(x, o);

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
	commands[0] = x.max_step * ay;
	commands[1] = x.max_step * ax;
	commands[2] = 0.0;

	// preserve current action as last to check for direction
	last[0] = ax;
	last[1] = ay;

	// We keep track of vehicle's movement here
	x.move(ax, ay);

	return commands;
}
