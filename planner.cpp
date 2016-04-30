#include <cstdlib>
#include <unistd.h>
#include <limits.h>

#include "../bearing/bearing.h"   //only change by ld
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


Planner::~Planner() {

}


void Planner::reset_observations() {
	
	/* clear the vectors to get ready for another observation set */
	_angles.clear();
	_gains.clear();
	_omni_gains.clear();
	_norm_gains.clear();

}


void Planner::complete_observations() {

	/* get bearing and values */
	_bearing_cc = get_bearing_cc(_angles, _gains);		// do bearing calculation at this point
	_bearing_max = get_bearing_max(_angles, _gains);	// also do max bearing calculation
	_max_rssi = get_max_rssi(_gains);					// get what the max value was for the rssi

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

	/* add heading and rssi to the correct arrays */
	_angles.push_back((double) heading);
	_gains.push_back(dir_gain);
	_omni_gains.push_back(omni_gain);

}

