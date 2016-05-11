/**
 * @file fixed_planner.cpp
 *
 * definition of the FixedPlanner class.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <inttypes.h>
#include <unistd.h>
#include <stdio.h>

#include "fixed_planner.h"

using namespace std;

FixedPlanner::FixedPlanner(string logfile_dir, const char * command_file_name) : Planner(),
_command_file_name(command_file_name),
_num_cmds(0),
_cmd_index(0)
{
	_cmd_north.clear();
	_cmd_east.clear();
	_cmd_yaw.clear();
	_cmd_alt.clear();

	// open the logfile
	std::string output_logfile_name = logfile_dir + "fixed_planner.log";
	_logfile = fopen(output_logfile_name.c_str(), "a");
}


FixedPlanner::~FixedPlanner() {
	fclose(_logfile);
}


int FixedPlanner::initialize() {

	// load up all the commands from the file
	string file_name(_command_file_name);
	ifstream cmd_file (file_name);

	if (!cmd_file.is_open()) {
		// means there is an error in loading the file
		return -1;
	}

	// make sure vectors are clean
	_cmd_north.clear();
	_cmd_east.clear();
	_cmd_yaw.clear();
	_cmd_alt.clear();

	float cmdN;
	float cmdE;
	float cmdY;
	float cmdA;
	char comma;
	while (cmd_file >> cmdN >> comma >> cmdE >> comma >> cmdY >> comma >> cmdA) {
		_cmd_north.push_back(cmdN);
		_cmd_east.push_back(cmdE);
		_cmd_yaw.push_back(cmdY);
		_cmd_alt.push_back(cmdA);

		printf("[FIXED PLANNER] North command: %f\n", cmdN);
		fprintf(_logfile, "North command: %f\n", cmdN);
		printf("[FIXED PLANNER] East cmd: %f\n", cmdE);
		fprintf(_logfile, "East cmd: %f\n", cmdE);
		printf("[FIXED PLANNER] Yaw cmd: %f\n", cmdY);
		fprintf(_logfile, "Yaw cmd: %f\n", cmdY);
		printf("[FIXED PLANNER] Alt cmd: %f\n", cmdA);
		fprintf(_logfile, "Alt cmd: %f\n", cmdA);
	}
	
	_num_cmds = _cmd_north.size();
	printf("[FIXED PLANNER] num commands read: %d\n", _num_cmds);
	fprintf(_logfile, "num commands read: %d\n", _num_cmds);
	
	return 0;
}


Action FixedPlanner::action() {

	// cycle the cmds (ids should go from 0 -> 3)
	if (_cmd_index >= _num_cmds) {
		_cmd_index = 0;
	}

	printf("[FIXED PLANNER] sending move command with index: %d\n", _cmd_index);
	fprintf(_logfile, "sending move command with index: %d\n", _cmd_index);

	// extract the next north and east commands
	float nextNorth = _cmd_north[_cmd_index];
	float nextEast = _cmd_east[_cmd_index];
	float nextYaw = _cmd_yaw[_cmd_index];
	float nextAlt = _cmd_alt[_cmd_index];

	printf("[FIXED PLANNER] sending command %i: N %f\tE %f\tY %f\tA %f\n", _cmd_index, nextNorth, nextEast, nextYaw, nextAlt);
	fprintf(_logfile, "sending command %i: N %f\tE %f\tY %f\tA %f\n", _cmd_index, nextNorth, nextEast, nextYaw, nextAlt);

	_cmd_index++;


	Action action = {};
	Action::set_basic_motion(&action, nextNorth, nextEast, nextYaw, nextAlt);

	// return the next command to be used
	return action;
}
