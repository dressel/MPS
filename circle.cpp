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
	vector<float>commands;
	commands.push_back(10.0 * ay);
	commands.push_back(10.0 * ax);

	// preserve current action as last to check for direction
	last[0] = ax;
	last[1] = ay;

	return commands;
}
