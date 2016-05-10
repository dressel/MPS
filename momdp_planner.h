#ifndef MOMDP_H_
#define MOMDP_H_

#include "myplanner.h"

class AlphaVector
{
	public:
		int a;
		vector<float> vec;
		AlphaVector(int a, vector<float> vec);
};

class MOMDPPlanner : public MyPlanner
{
	public:
		/**
		 * Constructor
		 */
		MOMDPPlanner(string paramfile, string logpath);
		vector<float> get_action();
		int initialize();

		vector< vector<AlphaVector>> policy;	// alpha vectors
		int _n;									// cells per side

	private:
		int _x;		//vehicle positions (in grid cells, not actual)
		int _y;
};

#endif
