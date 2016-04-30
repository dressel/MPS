#include "circle.h"

CirclePlanner::CirclePlanner(Vehicle x, Filter *f)
{
	this->x = x;
	this->filter = f;
	// handle last action
	this->last.clear();
	this->last.push_back(0.0);
	this->last.push_back(0.0);
}

bool CirclePlanner::initialize()
{
	return true;
}

vector<float> CirclePlanner::action()
{
	//double o = 45.0;
	//double o = this->_bearing_max;
	filter->update(x, _bearing_max);
	//filter->update(x, o);
	filter->print_belief();

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
	vector<float>commands (2);
	commands[0] = 10.0 * ay;
	commands[1] = 10.0 * ax;

	// preserve current action as last to check for direction
	last[0] = ax;
	last[1] = ay;

	// We keep track of vehicle's movement here
	x.x += ax;
	x.y += ay;

	return commands;
}
