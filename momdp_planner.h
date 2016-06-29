#ifndef MOMDP_H_
#define MOMDP_H_

#include "myplanner.h"

class AlphaVector
{
	public:
		int a;
		vector<float> vec;
		AlphaVector(int a, vector<float> vec);
		AlphaVector(int a);
};

class MOMDPPlanner : public MyPlanner
{
	public:
		/**
		 * Constructor
		 */
		MOMDPPlanner(string paramfile, string logpath);
		Action get_action();
		int initialize();

		vector< vector<AlphaVector>> policy;	// alpha vectors
		int _n;									// cells per side

	private:
		int _x;		//vehicle positions (in grid cells, not actual)
		int _y;
		int check_momdp_streams(ifstream &a_file, ifstream &o_file, ifstream &v_file);
};

#endif
