#ifndef GREEDY2_H_
#define GREEDY2_H_

#include "myplanner.h"

class GreedyPlanner2 : public MyPlanner
{
	public:

		/* constructor */
		GreedyPlanner2(string paramfile, string logfile);

		/* required functions for a MyPlanner subclass */
		int initialize();
		Action get_action();

	private:

		/* values that describe search grid */
		int _numcells_x;		// cells per side
		int _numcells_y;		// cells per side
		double _cellsize_x;		// size of a cell
		double _cellsize_y;		// size of a cell
		double _halfcell_x;		// size of half a cell
		double _halfcell_y;		// size of half a cell

		/* sets the above values using the base class's Vehicle object */
		int determine_search_area();

		/* cool stuff */
		Action get_action_slow();
		Action get_action_fast();
		Action get_action_faster(double (*)(double,double,double,double));
		Action get_action_no_obs();
		int initialize_non_obs_actions(string non_obs_path);

		/* some optional stuff */
		vector<vector<double> > non_obs_actions;
		int non_obs_index;

};
#endif
