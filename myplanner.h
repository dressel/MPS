#ifndef PLANNER_H_
#define PLANNER_H_

#include <string>
#include <sstream>
#include <fstream>
#include "planner.h"
#include "filter/df.h"

using std::string;
using std::ifstream;
using std::stringstream;

class MyPlanner : public Planner
{
	public:
		MyPlanner();
		MyPlanner(string paramfile, string logpath);
		Vehicle _uav;
		Filter *filter;
		string _param_file;
		string _log_path;
		std::ofstream planner_log;
		double _search_size;
		double get_obs();
		virtual int initialize() {};
		virtual vector<float> action() {};

	//protected:
		int read_param_line(string line, string path);
		int read_search_size_line(string line);
		int read_sensor_line(string line, string path);
		int read_filter_line(string line);

		/**
		 * If there is an error, it will return the string "error"
		 */
		string read_config(string paramfile);
};
#endif
