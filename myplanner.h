#ifndef PLANNER_H_
#define PLANNER_H_

#include "planner.h"
#include "filter/df.h"

class MyPlanner : public Planner
{
	public:
		Vehicle x;
		Filter *filter;
		//virtual vector action() = 0;
};
#endif
