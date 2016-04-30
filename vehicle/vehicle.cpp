#include "vehicle.h"

Vehicle::Vehicle()
{
	this->x = 5.0;
	this->y = 5.0;
	this->heading = 0.0;
	this->limit = 10.0;
	this->max_step = 2.0;
	//BearingOnly temp = new BearingOnly();
	//this->sensor = &temp;
	this->sensor = new BearingOnly();
}

// TODO: actually check the bounds here
vector<double> Vehicle::new_pose(vector<float> a)
{
	double new_x = x + a[0];
	double new_y = y + a[1];
	vector<double> temp (2);
	temp[0] = new_x;
	temp[1] = new_y;
	return temp;
}
