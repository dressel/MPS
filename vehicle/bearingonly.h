#ifndef VEHICLE_BEARINGONLY_H_
#define VEHICLE_BEARINGONLY_H_

#include "sensor.h"

class BearingOnly : public Sensor
{
	public:
		int noise;
		BearingOnly();
		~BearingOnly();
		int type();
};
#endif
