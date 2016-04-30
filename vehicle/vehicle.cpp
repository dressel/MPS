#include "vehicle.h"

Vehicle::Vehicle()
{
	this->x = 5.0;
	this->y = 5.0;
	this->heading = 0.0;
	//BearingOnly temp = new BearingOnly();
	//this->sensor = &temp;
	this->sensor = new BearingOnly();
}
