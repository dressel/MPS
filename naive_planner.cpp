/**
 * @file naive_planner.cpp
 *
 * definition of the naive planner class.
 * 
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#include <cstdlib>
#include <fstream>
#include <cmath>
#include <vector>
#include <inttypes.h>
#include <unistd.h>

#include "naive_planner.h"

using std::vector;

#define BEARING_TOL 10.0			// desired tolerance in degrees for 2 bearing measurements to be considered the same
#define STEP_INCREASE_FACTOR 2.0	// factor by which step size increases if along the same path as before
#define STEP_SMALL 10.0
#define STEP_LARGE 50.0


NaivePlanner::NaivePlanner(std::string logfile_dir) : Planner(),
_first_step(true)
{
	_observed_bearing.clear();
	_observed_rssi.clear();
	_step_sizes.clear();

	// open the logfile
	std::string output_logfile_name = logfile_dir + "naive_planner.log";
	_logfile = fopen(output_logfile_name.c_str(), "a");
}

NaivePlanner::~NaivePlanner() {
	fclose(_logfile);
}


void NaivePlanner::update_naive_observations() {

	printf("[NAIVE] updating the naive observation with bearing %f and max rssi %f\n", _bearing_max, _max_rssi);
	fprintf(_logfile, "[NAIVE] updating the naive observation\n");

	/* simply adding the most recent bearing and max rssi to the list of observations we've made */
	_observed_bearing.push_back(_bearing_max);
	_observed_rssi.push_back(_max_rssi);

}


float NaivePlanner::calculate_step_size() {
	// figure out how many observations have been made
	int numObs = _observed_bearing.size();

	float prevStepSize = _step_sizes[_step_sizes.size() - 1];
	float nextStepSize = prevStepSize;

	float difference = abs(_observed_bearing[numObs-1] - _observed_bearing[numObs-2]); 
	printf("[NAIVE] difference is %f\n", difference);
	fprintf(_logfile, "difference is %f\n", difference);
	if (difference < BEARING_TOL) {
		printf("[NAIVE] step sizes within tolerance\n");
		fprintf(_logfile, "[NAIVE] step sizes within tolerance\n");
		// double the step size
		nextStepSize *= STEP_INCREASE_FACTOR;
	} else {
		printf("[NAIVE] step size not within tol\n");
		fprintf(_logfile, "step size not within tol\n");
		// reset back to the smallest step size
		nextStepSize = STEP_SMALL;
	}

	return nextStepSize;
}


vector<float> NaivePlanner::calc_next_command(const double &bearing, const double &rssi) {
	// bearing is degrees from 0 to 359

	printf("[NAIVE] calculating the next command with input (%f, %f)\n", bearing, rssi);
	fprintf(_logfile, "calculating the next command with input (%f, %f)\n", bearing, rssi);
	
	// for debug purposes, force a specific bearing and rssi
	double bear = _bearing_max;
	if (bear >= 360.0) {
		bear = bear - 360.0;
	}
	int rs = 20;

	// commands are a vector of [north, south]
	vector<float> commands;

	float k = 0.5; // units: m / dB

	//float north = k * (double) rssi * cos(bearing * M_PI/180.0);
	//float east = k * ( double) rssi * sin(bearing * M_PI/180.0);

	float north = k * (double) rs * cos(bear * M_PI/180.0);
	float east = k * ( double) rs * sin(bear * M_PI/180.0);

	commands.push_back(north);	// dNorth
	commands.push_back(east);	// dEast
	commands.push_back(0);		// dYaw
	commands.push_back(-1);		// altitude

	return commands;
}


Action NaivePlanner::calc_next_command_variable(const double &bearing, const double &rssi) {

	float step = STEP_SMALL;
	if (!_first_step) {
		step = calculate_step_size();
	} else {
		_first_step = false;
	}
	_step_sizes.push_back(step);

	printf("[NAIVE] using step size %f\n", step);
	fprintf(_logfile, "using step size %f\n", step);

	printf("[NAIVE] calculating the next variable size command with input (%f, %f)\n", bearing, rssi);
	fprintf(_logfile, "calculating the next variable size command with input (%f, %f)\n", bearing, rssi);

	// commands are a vector of [north, south]
	vector<float> commands;

	float north = step * cos(bearing * M_PI/180.0);
	float east = step * sin(bearing * M_PI/180.0);

	Action action{};
	Action::set_relative_motion(&action, north, east);

	return action;
}

// the virtual function implementations

int NaivePlanner::initialize() {
	// nothing needs to be done for initialization.... this just needs to be here
	
	return 0;
}


Action NaivePlanner::action() {
	// use the most recently calculated bearing and max rssi to update the internal lists
	update_naive_observations();

	// TODO: need some configurability...

	// right now just use the variable step size code
	return calc_next_command_variable(_bearing_max, _max_rssi);
}
