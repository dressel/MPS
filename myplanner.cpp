#include "myplanner.h"

/* for debug purposes */
using std::cout;
using std::endl;


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
	int stype = _uav.sensor->type();
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
string MyPlanner::read_config(string paramfile)
{
	ifstream param_stream;
	string line;
	param_stream.open(paramfile);
	if (!param_stream.is_open())
	{
		std::cout << "FAILURE TO OPEN PLANNER CONFIGURATION FILE.\n";
		return "error";
	}

	/* get the path */
	int num_chars = paramfile.find_last_of("/") + 1;
	string path = paramfile.substr(0, num_chars);

	/* read each line */
	while( !getline(param_stream, line).eof() )
		read_param_line(line);


	/* close the stream and return the path of configuration files */
	param_stream.close();
	return path;
}

/**
 * Returns error if the line has some error in it
 */
bool MyPlanner::read_param_line(string line)
{
	//cout << "rpl = " << line << endl;
	if (line.substr(0,1) == "#")	// line is a comment
		return true;

	if (line.find("search_size") != string::npos)
		return read_search_size_line(line);
	if (line.find("sensor") != string::npos)
		return read_sensor_line(line);
	if (line.find("filter") != string::npos)
		return read_filter_line(line);

	// it's ok if we don't recognize the line
	return true;
}

bool MyPlanner::read_search_size_line(string line)
{
	int ind = line.find_last_of(",")+1;
	_search_size = stod(line.substr(ind, line.length()-ind));
	_uav.set_limit(_search_size);
	_uav.set_xy();
	_uav.set_max_step(4);	// TODO: do this correctly
}
bool MyPlanner::read_sensor_line(string line)
{
	string sub;
	stringstream ss(line);
	getline(ss, sub, ',');
	getline(ss, sub, ',');		// done twice to get second element
	if (sub == "bo")
	{
		_uav.sensor = new BearingOnly();
		return true;
	}
	if (sub == "do")
	{
		_uav.sensor = new DirOmni("ha", "ra");
		return true;
	}
	// log failure to recognize sensor
	return false;
}

bool MyPlanner::read_filter_line(string line)
{
	int filter_info = 11;
	this->filter = new DF(_search_size, filter_info);
}
