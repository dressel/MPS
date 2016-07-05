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
		~MyPlanner();
		Vehicle _uav;
		Filter *filter;
		string _param_file;
		string _log_path;
		double _search_size;

		virtual int initialize() {};
		Action action();


	protected:
		FILE *_plannerlog;
		int start_log();
		string read_config(string paramfile);
		string read_config2(string paramfile);
		string read_config_safe(string paramfile);
		void log_config();
		int _policy_extra_1;

		/**
		 * action = d_north,d_east,d_yaw (in meters)
		 * observation = o
		 */
		void print_action(vector<float> &a);
		void print_obs(double o);
		void print_action(Action a);

		void update_belief();
		virtual Action get_action() {};


	private:
		double get_obs();
		int read_param_line(string line, string path);
		int read_search_size_line(string line);
		int read_sensor_line(string line, string path);
		int read_filter_line(string line);
		int read_policy_line(string line);

};
#endif
