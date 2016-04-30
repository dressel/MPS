#ifndef VEHICLE_VEHICLE_H_
#define VEHICLE_VEHICLE_H_

#include "sensor.h"
#include "bearingonly.h"

class Vehicle
{
	public:
		double x;
		double y;
		double heading;
		Sensor *sensor;

		Vehicle();
};
#endif //VEHICLE_VEHICLE_H_
