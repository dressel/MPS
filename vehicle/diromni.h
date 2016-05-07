#ifndef VEHICLE_DIROMNI_H_
#define VEHICLE_DIROMNI_H_

#include <vector>
#include <string>
#include <fstream>
#include "sensor.h"

using std::vector;
using std::string;
using std::ifstream;

// for debugging
#include <iostream>
using std::cout;
using std::endl;

class DirOmni : public Sensor
{
	public:
		DirOmni();
		int type();
		bool set_stds(string path);
		bool set_means(string path);

	//private:
		vector<double> _means;
		vector<double> _stds;
};
#endif
