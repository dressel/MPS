#include "myplanner.h"
#include "vehicle/diromni.h"

using std::cout;
using std::endl;

MyPlanner::MyPlanner()
{
	//filter = NULL;
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
	fprintf(_plannerlog, "observation = %.2f\n", o);
	fflush(_plannerlog);
}
void MyPlanner::print_action(Action a)
{
	fprintf(_plannerlog, "action (north,east,yaw) = %.2f,%.2f,%.2f\n", a.north, a.east, a.yaw);
	fflush(_plannerlog);
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


/**
 * Reads in a configuration file and creates the vehicle and filter
 * Should fail gracefully
 */
string MyPlanner::read_config(string paramfile)
{
	_policy_extra_1 = 0;
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
	log_config();
	return path;
}

void MyPlanner::log_config()
{
	fprintf(_plannerlog, "CONFIG SUMMARY:\n");

	// determine search size
	fprintf(_plannerlog, "\tsearch size = %.1f\n", _search_size);

	// vehicle stuff
	fprintf(_plannerlog, "\tvehicle = %.1f\n", _search_size);
	fprintf(_plannerlog, "\t\txmin = %.1f\n", _uav._xmin);
	fprintf(_plannerlog, "\t\txmax = %.1f\n", _uav._xmax);
	fprintf(_plannerlog, "\t\tymin = %.1f\n", _uav._ymin);
	fprintf(_plannerlog, "\t\tymax = %.1f\n", _uav._ymax);
	
	// determine filter type
	fprintf(_plannerlog, "\tfilter type = %d\n", filter->type);
	if (filter->type == 0)
	{
		DF* df = static_cast<DF*>(filter);
		fprintf(_plannerlog, "\t\tn = %d\n", df->n);
	}

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

	if (line.find("vehicle") != string::npos)
		return read_vehicle_line(line);
	
	if (line.find("sensor") != string::npos)
		return read_sensor_line(line, path);
	
	if (line.find("filter") != string::npos)
		return read_filter_line(line);

	if (line.find("policy") != string::npos)
		return read_policy_line(line);
	

	// it's ok if we don't recognize the line
	// TODO: really, we should be upset we didn't recognize the line
	return 0;
}

int MyPlanner::read_policy_line(string line)
{
	int ind = line.find_last_of(",")+1;
	_policy_extra_1 = stoi(line.substr(ind, line.length()-ind));
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

//TODO: do some error checking here
int MyPlanner::read_vehicle_line(string line)
{
	string sub;
	stringstream ss(line);
	getline(ss, sub, ','); // getting rid of first element

	while (getline(ss,sub,','))
	{
		if (sub == "xy")
		{
			getline(ss, sub, ',');
			double x = stod(sub);
			getline(ss, sub, ',');	
			double y = stod(sub);
			_uav.set_xy(x,y);
			printf("x = %.3f, y = %.3f\n", x,y);
		}
		if (sub == "limits")
		{
			getline(ss, sub, ',');
			int xmin = stod(sub);
			getline(ss, sub, ',');
			int xmax = stod(sub);
			getline(ss, sub, ',');
			int ymin = stod(sub);
			getline(ss, sub, ',');
			int ymax = stod(sub);
			_uav.set_limits(xmin,xmax,ymin,ymax);
		}
		if (sub == "cells")
		{
			getline(ss, sub, ',');
			_uav._numcells_x = stoi(sub);
			getline(ss, sub, ',');
			_uav._numcells_y = stoi(sub);
		}
	}

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
		_uav.sensor = new BearingOnly(13.0);
		fprintf(_plannerlog, "New BearingOnly sensor created.\n");
		fflush(_plannerlog);
		return 0;
	}
	else if (sub == "do")
	{
		// TODO: Ahhhh this is ugly. This shouldn't be here.
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
		// get size
		getline(ss, sub, ',');
		DF* df = new DF(_search_size, stoi(sub));

		// is it a fast filter or a slow one?
		getline(ss, sub, ',');
		// TODO: allow user to enter nothing here
		if (stoi(sub) == 1)
		{
			//fast
			printf("setting obs probs for fast discrete filter\n");
			df->set_obs_probs(_uav.sensor);
		}
		filter = df;

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
	fprintf(_plannerlog, "belief\n");
	filter->print_belief(_plannerlog);
}
