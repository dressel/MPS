#include "myplanner.h"


MyPlanner::MyPlanner()
{
}



/**
 * Uses the sensor type of the planner's vehicle to determine
 *  what constitutes an observation.
 *
 */
double MyPlanner::get_obs()
{
	double o = 0.0;
	int stype = x.sensor->type();
	if (stype == 0)
		o = _bearing_max;
	else if (stype == 1)
		o = 0.0;	//TODO: let's add the other sensor...
	else
		std::cout << "ERROR: SENSOR TYPE UNRECOGNIZED" << std::endl;
	return o;
}

/**
 * Reads in a configuration file and creates the vehicle and filter
 * Should fail gracefully
 */
bool MyPlanner::read_config(string paramfile)
{
	ifstream params;
	string line;
	params.open(paramfile);
	if (!params.is_open())
		std::cout << "FAILURE TO OPEN PLANNER CONFIGURATION FILE." << std::endl;

	// first param will be size of domain;
	getline(params, line);
	double search_size = stod(line);
	//std::cout << "search_size = " << search_size << std::endl;

	/* Sensor and vehicle */
	// second is sensor type, third is max_step size for vehicle
	getline(params, line);
	int sensor_type = stoi(line);
	if (sensor_type == 0)
		this->x.sensor = new BearingOnly();

	getline(params, line);
	int max_step = stoi(line);
	this->x.set_limit(search_size);
	this->x.set_max_step(max_step);
	this->x.set_xy();

	/* Filter */
	// fourth is filter type, fifth is secondary filter
	getline(params, line);
	int filter_type = stoi(line);
	getline(params, line);
	int filter_info = stoi(line);
	if (filter_type == 0)
		this->filter = new DF(search_size, filter_info);

	//TODO: actually make the vehicle correctly
	//this->x = Vehicle();
	//this->x.max_step = max_step;

	params.close();
	return true;
}
