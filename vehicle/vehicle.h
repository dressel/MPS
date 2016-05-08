#ifndef VEHICLE_VEHICLE_H_
#define VEHICLE_VEHICLE_H_

#include <vector>
#include "sensor.h"
#include "bearingonly.h"
#include "diromni.h"

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
		Vehicle(double search_size, double max_step);
		void set_limit(double search_size);
		void set_max_step(double max_step);
		void set_xy();
		void set_xy(double x, double y);

		/**
		 * Returns a 3-element vector of doubles.
		 */
		vector<double> new_pose(vector<float> &a);
		void move(float ax, float ay);
		void move(float ax, float ay, float ah);
};
#endif //VEHICLE_VEHICLE_H_
