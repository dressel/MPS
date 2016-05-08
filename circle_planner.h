#ifndef CIRCLE_H_
#define CIRCLE_H_

#include "myplanner.h"

class CirclePlanner : public MyPlanner
{
	public:
		//CirclePlanner(Vehicle x, Filter *f);
		CirclePlanner(string paramfile);
		int initialize();
		vector<float> action();
		vector<float> last;
};
#endif
