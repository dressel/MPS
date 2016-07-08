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

		/* constructors, destructor */
		MyPlanner();
		MyPlanner(string paramfile, string logpath);
		~MyPlanner();

		Vehicle _uav;
		Filter *filter;
		string _param_file;
		string _log_path;
		double _search_size;

		/* required of any Planner subclass */
		virtual int initialize() {};
		Action action();


	protected:
		FILE *_plannerlog;
		int start_log();
		string read_config(string paramfile);
		int _policy_extra_0;
		int _policy_extra_1;

		/**
		 * functions that print the following:
		 *   action = d_north,d_east,d_yaw (in meters)
		 *   observation = o
		 */
		void print_action(Action a);
		void print_obs(double o);
		void print_meancov();

		/**
		 * Each function has its individual get_action function
		 * This function is called by MyPlanner's `action` function
		 * get_action_no_obs defaults to get_action
		 */
		virtual Action get_action() {};
		virtual Action get_action_no_obs() {return get_action();};


	private:
		/* Determines observation, updates belief, and prints them */
		void update_belief();

		/**
		 * Uses the sensor type of the planner's vehicle to determine
		 *  what constitutes an observation.
		 */
		double get_obs();

		/* read lines of various types from config file and set fields */
		int read_param_line(string line, string path);
		int read_search_size_line(string line);
		int read_vehicle_line(string line);
		int read_sensor_line(string line, string path);
		int read_filter_line(string line);
		int read_policy_line(string line);

		/* log our configuration choices */
		void log_config();
};
#endif
