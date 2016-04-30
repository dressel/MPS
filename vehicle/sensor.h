#ifndef SENSOR_H_
#define SENSOR_H_

// 0 if bearing only
// 1 if dir omni
class Sensor
{
	public:
		virtual int type() = 0;
};
#endif	// SENSOR_H_
