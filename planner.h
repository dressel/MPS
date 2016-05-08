/**
 * @file planner.h
 *
 * Declaration of superclass for all planners.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef _PLANNER_H_
#define _PLANNER_H_


#include <vector>
using std::vector;

/**
 * @class Planner
 * 
 * This is the superclass for any of the planners.
 * Sets all of the observable information and is a passthrough for the action functions.
 *
 * Any subclass must implement and initialize() and action() functions.
 * 
 */
class Planner {

public:

	/**
	 * constructor
	 */
	Planner();

	/**
	 * destructor
	 */
	~Planner();

	/**
	 * reset the current observation values.  Not necessarily needed, but good to make sure.
	 */
	void reset_observations();

	/**
	 * determine max signal strength from a set of measurements.
	 * @param  rssi_values  vector of the set of measurements
	 * @return              integer value of the maximum signal strength
	 */
	int get_max_rssi(const vector<double> rssi_values);

	/**
	 * update a single observation.  A single observations is just a heading and signal strength.
	 * @param heading   heading of the antenna for this measurement
	 * @param dir_gain  signal strength value from the directional antenna
	 * @param omni_gain signal strength value from the omni directional atenna
	 */
	void update_observation(const double &heading, const double &dir_gain, const double &omni_gain);

	/**
	 * update a set of observations.  For example after a rotation.
	 * @param headings     vector of headings for the signal strength measurements
	 * @param dir_gains    vector of measured signal strength values from the directional antenna
	 * @param omni_gains   vector of measured signal strength values from the omnidirectional antenna
	 * @param bearing_cc   calculated cross correlation bearing
	 * @param bearing_max  calculated max bearing
	 * @param bearing_max3 calculated max3 bearing
	 */
	void update_observations(const vector<double> headings, const vector<double> dir_gains, const vector<double> omni_gains, const vector<int> norm_gains,
							 const double &bearing_cc, const double &bearing_max, const double &bearing_max3);

	/**
	 * initialize the current planner. To be implemented by each individual planner.
	 * @return -1 if initialization fails
	 */
	virtual int initialize() {};

	/**
	 * calculates what the next action should be.  This is to be each individual planner.
	 * @return  action as a vector, defined as <dNorth, dEast, dYaw, alt>
	 */
	virtual vector<float> action() {};

protected:

	/* constants that will be available to all the subclasses */
	vector<double> _angles;		// the heading associated with the gains
	vector<double> _gains;		// the gains measured from the main sensor (e.g. directional antenna)
	vector<double> _omni_gains;	// the gains measured from the second sensor (e.g. omni antenna)
	vector<int> _norm_gains;	// the normalized gains
	double _bearing_cc;			// the cross correlation bearing
	double _bearing_max;		// the max method bearing
	double _bearing_max3;		// the max3 method bearing
	double _max_rssi;			// the max signal strength in this set
};


#endif	/* _PLANNER_H_ */
