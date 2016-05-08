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
		vector< vector<AlphaVector>> policy;
		MOMDPPlanner(string paramfile);
		vector<float> action();
		int initialize();
		int _n;		// number of cells per side
		vector<vector<float> > actions;

	private:
		int _x;		//vehicle positions (in grid cells, not actual)
		int _y;
};

#endif
