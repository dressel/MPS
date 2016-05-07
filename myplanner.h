#ifndef PLANNER_H_
#define PLANNER_H_

#include <string>
#include <fstream>
#include "planner.h"
#include "filter/df.h"

using std::string;
using std::ifstream;

class MyPlanner : public Planner
{
	public:
		MyPlanner();
		Vehicle x;
		Filter *filter;
		double get_obs();
		virtual bool initialize() {};
		virtual vector<float> action() {};
		string read_config(string paramfile);
};
#endif
