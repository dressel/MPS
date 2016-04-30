#ifndef _PLANNER_H_
#define _PLANNER_H_


#include <vector>
using std::vector;

/**
 * this is the superclass for any of the planners.
 * 
 */
class Planner {

public:

	/* constructor */
	Planner();

	/* destructor */
	~Planner();

	/* reset set of obs (e.g. starting a rotation) */
	void reset_observations();

	/* complete a set of observations (e.g. ending a rotation) */
	void complete_observations();

	/* determine what the max rssi value was of the last rotation set */
	int get_max_rssi(const vector<double> rssi_values);

	/* update with a single observation */
	// TODO: maybe want lat/lon here....?
	void update_observation(const double &heading, const double &dir_gain, const double &omni_gain);

	/* initialize the current planner */
	// TODO: these initializations might need different inputs....
	virtual bool initialize() {};

	/* calculates what the next action should be - this is to be implemented by the subclasses */
	virtual vector<float> action() {};

protected:

	// constants that will be available to all the subclasses
	vector<double> _angles;		/* the heading associated with the gains */
	vector<double> _gains;		/* the gains measured from the main sensor (e.g. directional antenna) */
	vector<double> _omni_gains;	/* the gains measured from the second sensor (e.g. omni antenna) */
	vector<int> _norm_gains;	/* the normalized gains */
	double _bearing_cc;			/* the cross correlation bearing */
	double _bearing_max;		/* the max method bearing */
	double _bearing_max3;		/* the max3 method bearing */
	double _max_rssi;			/* the max signal strength in this set */
};


#endif	/* _PLANNER_H_ */