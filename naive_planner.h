/**
 * @file naive_planner.h
 *
 * Declaration of naive planning class.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef _NAIVE_PLANNER_H_
#define _NAIVE_PLANNER_H_

#include "planner.h"


/**
 * @class NaivePlanner
 *
 * A greedy planner that moves in the direction of the most recent bearing measurement.
 * Can move with a variable step size based on previous measurements.
 * 
 */
class NaivePlanner : public Planner {

public:

	/**
	 * constructor
	 */
	NaivePlanner();

	/**
	 * desctructor
	 */
	~NaivePlanner();

	/**
	 * initialize the planner
	 * @return true if successful
	 */
	bool initialize();

	/**
	 * determins the next action to take.
	 * @return  vector of <dNorth, dEast, dYaw> action values
	 */
	vector<float> action();

private:

	// overall information used to make decisions
	vector<double> _observed_bearing;
	vector<int> _observed_rssi;
	vector<float> _step_sizes;

	bool _first_step;


	/**
	 * update our local observation list of all the bearings and max rssi values so far.
	 */
	void update_naive_observations();

	/**
	 * calculate the next step size.
	 * @return the step size (in meters) that should be used
	 */
	float calculate_step_size();

	/**
	 * calculate the next command with constant step size.
	 * @return  the command
	 */
	vector<float> calc_next_command(const double &bearing, const double &rssi);

	/**
	 * calculate the next command with a variable step size
	 * @return  the command
	 */
	vector<float> calc_next_command_variable(const double &bearing, const double &rssi);
};

#endif	/* _NAIVE_PLANNER_H_ */