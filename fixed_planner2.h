#ifndef FIXED2_H_
#define FIXED2_H_

#include "myplanner.h"

class FixedPlanner2 : public MyPlanner
{
	public:

		/* constructor */
		FixedPlanner2(string paramfile, string logfile);

		/* required functions for a MyPlanner subclass */
		int initialize();
		Action get_action();

	private:

		int initialize_actions(string non_obs_path);

		/* required for fixed planner */
		vector<vector<double> > actions;
		int action_index;
		int num_actions;

};
#endif
