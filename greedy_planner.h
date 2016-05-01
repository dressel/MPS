#ifndef GREEDY_H_
#define GREEDY_H_

#include "myplanner.h"

class GreedyPlanner : public MyPlanner
{
	public:
		//GreedyPlanner(Vehicle x, Filter *f, int n);
		GreedyPlanner(string paramfile);
		vector<float> action();
		int n;
		vector<vector<float> > actions;
		vector<float> find_best_action(Vehicle x);
};
#endif
