#ifndef CIRCLE_H_
#define CIRCLE_H_

#include "myplanner.h"

class CirclePlanner : public MyPlanner
{
	public:
		CirclePlanner(Vehicle x, Filter *f);
		int get_action(double o);
		vector<float> action();
		vector<float> last;
};
#endif
