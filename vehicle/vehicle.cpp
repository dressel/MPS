#include "vehicle.h"

Vehicle::Vehicle()
{
}

Vehicle::Vehicle(double ss, double ms)
{
	double search_size = 10.0;
	double max_step = 2.0;
	this->x = search_size / 2.0;
	this->y = search_size / 2.0;
	this->heading = 0.0;
	this->limit = ss;
	this->max_step = ms;
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

void Vehicle::set_xy()
{
	set_xy(limit/2.0, limit/2.0);
}

void Vehicle::set_xy(double px, double py)
{
	this->x = px;
	this->y = py;
}

void Vehicle::set_limit(double search_size)
{
	this->limit = search_size;
}

void Vehicle::set_max_step(double ms)
{
	this->max_step = ms;
}
