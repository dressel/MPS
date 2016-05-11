#ifndef CIRCLE_H_
#define CIRCLE_H_

#include "myplanner.h"

class CirclePlanner : public MyPlanner
{
	public:
		CirclePlanner(string paramfile, string logpath);
		int initialize();
		Action get_action();
		vector<float> last;
};
#endif
