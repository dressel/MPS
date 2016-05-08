#include "myplanner.h"
#include "vehicle/diromni.h"

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
	bool err_flag;
	while( !getline(param_stream, line).eof() )
	{
		err_flag = read_param_line(line, path);
		if (err_flag == true)
		{
			param_stream.close();
			return "error";
		}
	}


	/* close the stream and return the path of configuration files */
	param_stream.close();
	return path;
}

/**
 * Returns error if the line has some error in it
 */
bool MyPlanner::read_param_line(string line, string path)
{
	//cout << "rpl = " << line << endl;
	if (line.substr(0,1) == "#")	// line is a comment
		return false;

	if (line.find("search_size") != string::npos)
		return read_search_size_line(line);
	if (line.find("sensor") != string::npos)
		return read_sensor_line(line, path);
	if (line.find("filter") != string::npos)
		return read_filter_line(line);

	// it's ok if we don't recognize the line
	return false;
}

bool MyPlanner::read_search_size_line(string line)
{
	int ind = line.find_last_of(",")+1;
	_search_size = stod(line.substr(ind, line.length()-ind));
	_uav.set_limit(_search_size);
	_uav.set_xy();
	_uav.set_max_step(4);	// TODO: do this correctly
	return false;
}
bool MyPlanner::read_sensor_line(string line, string path)
{
	string sub;
	stringstream ss(line);
	getline(ss, sub, ',');
	getline(ss, sub, ',');		// done twice to get second element
	if (sub == "bo")
	{
		_uav.sensor = new BearingOnly();
		return false;
	}
	if (sub == "do")
	{
		bool erflag1 = true;
		DirOmni *domni = new DirOmni();
		string temp = path + "norm_means.csv";
		cout << "temp = " << temp << endl;
		//erflag1 = domni->set_means(path + "norm_means.csv");
		erflag1 = domni->set_means(temp);
		domni->set_stds(path + "norm_means.csv");
		_uav.sensor = domni;
		cout << "erflag1 = " << erflag1 << endl;
		// this is very ugly
		DF* f = static_cast<DF*>(filter);
		f->bin_offset = -20;
		return erflag1;
	}
	// log failure to recognize sensor
	return false;
}

bool MyPlanner::read_filter_line(string line)
{
	// TODO: don't hard code this
	int filter_info = 11;
	this->filter = new DF(_search_size, filter_info);
	return false;
}
