#include "vehicle.h"

//Vehicle::Vehicle(double search_size, double max_step)
Vehicle::Vehicle()
{
	double search_size = 10.0;
	double max_step = 2.0;
	this->x = search_size / 2.0;
	this->y = search_size / 2.0;
	this->heading = 0.0;
	this->limit = search_size;
	this->max_step = max_step;
	this->sensor = new BearingOnly();
}

// TODO: actually check the bounds here
/**
 * Recall that actions are north first, then east first
 */
vector<double> Vehicle::new_pose(vector<float> a)
{
	double new_y = y + a[0];
	double new_x = x + a[1];
	vector<double> temp (2);
	temp[0] = new_x;
	temp[1] = new_y;
	return temp;
}

void Vehicle::move(float ax, float ay)
{
	x += ax;
	y += ay;
}

void Vehicle::move(float ax, float ay, float ah)
{
	x += ax;
	y += ay;
	heading += ah;
}
