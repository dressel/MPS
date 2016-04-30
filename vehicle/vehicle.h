#ifndef VEHICLE_VEHICLE_H_
#define VEHICLE_VEHICLE_H_

#include <vector>
#include "sensor.h"
#include "bearingonly.h"

using std::vector;

class Vehicle
{
	public:
		double x;
		double y;
		double heading;
		double limit;		// size of region
		double max_step;
		Sensor *sensor;

		Vehicle();
		vector<double> new_pose(vector<float> a);
};
#endif //VEHICLE_VEHICLE_H_
