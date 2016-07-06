#ifndef VEHICLE_BEARINGONLY_H_
#define VEHICLE_BEARINGONLY_H_

#include "sensor.h"

class BearingOnly : public Sensor
{
	public:
		BearingOnly();
		BearingOnly(double noise_sigma);
		~BearingOnly();

		int type();
		int noise_sigma;	// std deviation, in degrees
};
#endif
