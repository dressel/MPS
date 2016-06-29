#include "myplanner.h"
#include "vehicle/diromni.h"

using std::cout;
using std::endl;

MyPlanner::MyPlanner()
{
}

MyPlanner::~MyPlanner()
{
	delete filter;
}


MyPlanner::MyPlanner(string paramfile, string logpath)
{
	_param_file = paramfile;
	_log_path = logpath;
}

Action MyPlanner::action()
{
	update_belief();
	Action a = get_action();
	print_action(a);
	_uav.move(a.east, a.north, a.yaw);
	return a;
}

void MyPlanner::print_obs(double o)
{
	fprintf(_plannerlog, "obs = %.2f\n", o);
	fflush(_plannerlog);
}
void MyPlanner::print_action(Action a)
{
	fprintf(_plannerlog, "COMMAND: (north,east,yaw) = %.2f,%.2f,%.2f\n", a.north, a.east, a.yaw);
	fflush(_plannerlog);
}

void MyPlanner::print_action(vector<float> &a)
{
	//planner_log << "COMMAND: (north,east,yaw) = " << a[0] << "," << a[1] << "," << a[2] << endl << endl;
}


/**
 *
 * Returns -1 (true) if failure, 0 otherwise.
 */
int MyPlanner::start_log()
{
	string temp = _log_path + "/planner_log.txt";
	_plannerlog = fopen(temp.c_str(), "a");
	if (_plannerlog == NULL)
	{
		// error opening file
		return -1;
	}
	fprintf(_plannerlog, "Planner is alive (MyPlanner::start_log)\n");
	fflush(_plannerlog);
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
	{
		o = _bearing_max;
	}
	else if (stype == 1)
	{
		o = 0.0;	//TODO: let's add the other sensor...
	}
	else
	{
		fprintf(_plannerlog, "ERROR: SENSOR TYPE UNRECOGNIZED\n");
		fflush(_plannerlog);
	}
	return o;
}


// I know the following works, but we want to move away from this


string MyPlanner::read_config_safe(string paramfile)
{
	/* get the path */
	int num_chars = paramfile.find_last_of("/") + 1;
	string path = paramfile.substr(0, num_chars);

	_search_size = 100.0;
	_uav.set_limit(_search_size);
	_uav.set_xy();
	_uav.heading = 0.0;
	_uav.set_max_step(10.0);	
	
	// TODO: do this correctly
	// the below seems to hang when I leave it be
	filter = new DF(_search_size, 41);

	// the below seems to work.
	_uav.sensor = new BearingOnly();

	return path;
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
		fprintf(_plannerlog, "FAILURE TO OPEN PLANNER CONFIG FILE.\n");
		fflush(_plannerlog);
		return "error";
	}

	/* get the path */
	int num_chars = paramfile.find_last_of("/") + 1;
	string path = paramfile.substr(0, num_chars);

	/* read each line */
	
	bool err_flag;
	
	/*
	while( !getline(param_stream, line).eof() )
	{
		err_flag = read_param_line(line, path);
		if (err_flag)
		{
			param_stream.close();
			return "error";
		}
	}
	*/
	
	
	read_config_safe(paramfile);


	/* close the stream and return the path of configuration files */
	param_stream.close();
	log_config();
	return path;
}

void MyPlanner::log_config()
{
	// determine search size
	fprintf(_plannerlog, "search size = %.1f\n", _search_size);
	
	// determine filter type
	fprintf(_plannerlog, "filter = \n");

	fflush(_plannerlog);
	return;
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
	// TODO: really, we should be upset we didn't recognize the line
	return 0;
}

int MyPlanner::read_search_size_line(string line)
{
	int ind = line.find_last_of(",")+1;
	_search_size = stod(line.substr(ind, line.length()-ind));
	_uav.set_limit(_search_size);
	_uav.set_xy();
	_uav.heading = 0.0;
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
		fprintf(_plannerlog, "New BearingOnly sensor created.\n");
		fflush(_plannerlog);
		return 0;
	}
	else if (sub == "do")
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

	fprintf(_plannerlog, "Failed to recognize requested sensor.\n");
	fflush(_plannerlog);
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
		/*
		int filter_info = 13;
		fprintf(_plannerlog, "sub = %s\n", sub.c_str());
		fflush(_plannerlog);
		*/
		int filter_info = 13;
		int filter_info2 = stoi(sub);
		int filter_info3 = atoi(sub.c_str());
		fprintf(_plannerlog, "sub = %d\n", filter_info);
		fprintf(_plannerlog, "sub = %d\n", filter_info2);
		fprintf(_plannerlog, "sub = %d\n", filter_info3);
		fflush(_plannerlog);
		filter = new DF(_search_size, filter_info3);
		fprintf(_plannerlog, "New discrete filter created.\n");
		fflush(_plannerlog);
		return 0;
	}

	fprintf(_plannerlog, "Failed to recognize requested filter.\n");
	fflush(_plannerlog);
	return -1;
}

void MyPlanner::update_belief()
{
	double o = get_obs();
	print_obs(o);
	filter->update(_uav, o);
	filter->print_belief(_plannerlog);
}
