#ifndef CIRCLE_H_
#define CIRCLE_H_

#include "myplanner.h"

class CirclePlanner : public MyPlanner
{
	public:

		/* constructor */
		CirclePlanner(string paramfile, string logpath);

		/* required functions for MyPlanner subclass */
		int initialize();
		Action get_action();

	private:
		vector<float> last;
};
#endif
