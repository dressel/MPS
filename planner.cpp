/**
 * @file planner.cpp
 *
 * definition of superclass for all planners.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#include <cstdlib>
#include <fstream>
#include <unistd.h>
#include <limits.h>

#include "planner.h"


Planner::Planner() :
_bearing_cc(0),
_bearing_max(0),
_bearing_max3(0),
_max_rssi(0)
{
	// make sure all the vectors have been cleared
	_angles.clear();
	_gains.clear();
	_omni_gains.clear();
	_norm_gains.clear();
}


Planner::~Planner() {}


void Planner::reset_observations() {
	
	// clear the vectors to get ready for another observation set
	_angles.clear();
	_gains.clear();
	_omni_gains.clear();
	_norm_gains.clear();
}


int Planner::get_max_rssi(const vector<double> rssi_values) {

	// set it to below what the RF detector can detect
	double max_rssi = -100;
	
	// loop through all values to get the max
	int len = rssi_values.size();
	for (int i = 0; i < len; i++) {
		
		// ignore any invalid measurement
		if (rssi_values[i] == INT_MAX) {
			continue;
		}

		// update max value
		if (rssi_values[i] > max_rssi) {
			max_rssi = rssi_values[i];
		}
	}

	// return the true max value
	return (int) max_rssi;
}


void Planner::update_observation(const double &heading, const double &dir_gain, const double &omni_gain) {

	// add heading and rssi to the correct arrays
	_angles.push_back((double) heading);
	_gains.push_back(dir_gain);
	_omni_gains.push_back(omni_gain);

	// TODO: probably want to call some sort of function implemented by each individual planner if want to be able to use this.
}


void Planner::update_observations(const vector<double> headings, const vector<double> dir_gains, const vector<double> omni_gains, const vector<int> norm_gains,
							 const double &bearing_cc, const double &bearing_max, const double &bearing_max3) {
	
	printf("[PLANNER] updating observation with bearings: %f, %f, %f\n", bearing_cc, bearing_max, bearing_max3);
	// copy over the vectors
	_angles = headings;
	_gains = dir_gains;
	_omni_gains = omni_gains;
	_norm_gains = norm_gains;

	// copy over the bearings
	_bearing_cc = bearing_cc;
	_bearing_max = bearing_max;
	_bearing_max3 = bearing_max3;

	// get the max signal strength for this set
	_max_rssi = get_max_rssi(_gains);
}
