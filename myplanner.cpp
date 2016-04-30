#include "myplanner.h"

MyPlanner::MyPlanner()
{
}

bool MyPlanner::initialize()
{
	return true;
}

double MyPlanner::get_obs()
{
	double o = 0.0;
	if (x.sensor->type() == 0)
		o = _bearing_max;
	return o;
}
vector<float> MyPlanner::action()
{
	vector<float> temp (2);
	temp[0] = 17.0;
	temp[1] = 17.0;
	return temp;
}
