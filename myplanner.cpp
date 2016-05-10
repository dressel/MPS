#include "myplanner.h"
#include "vehicle/diromni.h"

using std::cout;
using std::endl;


MyPlanner::MyPlanner()
{
}
MyPlanner::MyPlanner(string paramfile, string logpath)
{
	_param_file = paramfile;
	_log_path = logpath;
}

vector<float> MyPlanner::action()
{
	update_belief();
	vector<float> a = get_action();
	print_action(a);
	_uav.move(a);
	return a;
}

void MyPlanner::print_action(vector<float> &a)
{
	planner_log << "COMMAND: (north,east,yaw) = " << a[0] << "," << a[1] << "," << a[2] << endl;
}


/**
 *
 * Returns -1 (true) if failure, 0 otherwise.
 */
int MyPlanner::start_log()
{
	planner_log.open(_log_path + "/planner_log.txt");
	if (!planner_log.is_open())
		return -1;
	return 0;
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
		planner_log << "ERROR: SENSOR TYPE UNRECOGNIZED" << endl;
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
		planner_log << "FAILURE TO OPEN PLANNER CONFIGURATION FILE." <<endl;
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
		if (err_flag)
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
int MyPlanner::read_param_line(string line, string path)
{
	if (line.substr(0,1) == "#")	// line is a comment
		return 0;

	if (line.find("search_size") != string::npos)
		return read_search_size_line(line);
	if (line.find("sensor") != string::npos)
		return read_sensor_line(line, path);
	if (line.find("filter") != string::npos)
		return read_filter_line(line);

	// it's ok if we don't recognize the line
	return 0;
}

int MyPlanner::read_search_size_line(string line)
{
	int ind = line.find_last_of(",")+1;
	_search_size = stod(line.substr(ind, line.length()-ind));
	_uav.set_limit(_search_size);
	_uav.set_xy();
	_uav.set_max_step(10.0);	// TODO: do this correctly
	return 0;
}
int MyPlanner::read_sensor_line(string line, string path)
{
	string sub;
	int e_flag = 0;
	stringstream ss(line);
	getline(ss, sub, ',');
	getline(ss, sub, ',');		// done twice to get second element
	if (sub == "bo")
	{
		_uav.sensor = new BearingOnly();
		return 0;
	}
	if (sub == "do")
	{
		DirOmni *domni = new DirOmni();
		e_flag += domni->set_means(path + "norm_means.csv");
		e_flag += domni->set_stds(path + "norm_means.csv");
		_uav.sensor = domni;
		// TODO: check that filter is a discrete filter
		DF* f = static_cast<DF*>(filter);
		f->bin_offset = -20;
		return e_flag;
	}

	planner_log << "Failed to recognize requested sensor." << endl;
	return -1;
}

int MyPlanner::read_filter_line(string line)
{
	string sub;
	int e_flag = 0;
	stringstream ss(line);
	getline(ss, sub, ',');
	getline(ss, sub, ',');		// done twice to get second element

	if (sub == "df")
	{
		getline(ss, sub, ',');
		int filter_info = stoi(sub);
		filter = new DF(_search_size, filter_info);
		return 0;
	}

	planner_log << "Failed to recognize requested filter." << endl;
	return -1;
}

void MyPlanner::update_belief()
{
	double o = get_obs();
	filter->update(_uav, o);
	filter->print_belief(planner_log);
}
