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
 * @class Action
 *
 * The action that the planner desires.
 * Basically just contains a bunch of public fields.
 * This is a class instead of a struct, to be able to contain constants.
 */
class Action {

public:

	// flags that can be set to define the inputs
	static const int FLAG_ALTITUDE_ABSOLUTE = 0 << 0;
	static const int FLAG_ALTITUDE_RELATIVE = 1 << 0;

	static const int FLAG_YAW_RELATIVE = 0 << 1;
	static const int FLAG_YAW_ABSOLUTE = 1 << 1;

	static const int FLAG_EAST_RELATIVE = 0 << 2;
	static const int FLAG_EAST_ABSOLUTE = 1 << 2;

	static const int FLAG_NORTH_RELATIVE = 0 << 3;
	static const int FLAG_NORTH_ABSOLUTE = 1 << 3;

	// flags that can be set to determine which input have been set
	static const int VALID_ALTITUDE = 1 << 0;
	static const int VALID_EAST = 1 << 1;
	static const int VALID_NORTH = 1 << 2;
	static const int VALID_YAW = 1 << 3;
	static const int VALID_ROTATION_ANGLE = 1 << 4;
	static const int VALID_DIRECTION = 1 << 5;

	/** the north command in [m] */
	float north;

	/** the east command in [m] */
	float east;

	/** the yaw command in [deg] */
	float yaw;

	/** the altitude command in [m] */
	float altitude;

	/** the angle through which to rotate in [deg] [0, 360] */
	float rotation_angle;

	/** direction or rotation (1 = clockwise, -1 = counterclockwise) */
	int direction_of_rotation;

	/** 
	 * some flags to say what the command actually is made up of
	 * Starting from LSB:
	 * 0 - altitude absolute or relative
	 * 1 - yaw relative or absolute
	 * 2 - east relative or absolute
	 * 3 - north relative or absolute
	 */
	int flags;

	/**
	 * flags for which of the fields have valid data.
	 * Starting from LSB:
	 * 0 - altitude value
	 * 1 - yaw value
	 * 2 - east valid
	 * 3 - north valid
	 * 4 - rotation angle valid
	 * 5 - direction of rotation valid
	 */
	int valid;

	// some helper functions since there are no many flags
	
	/**
	 * set a basic relative North, East command that will execute a full rotation upon arrival
	 * @param action  the action to fill
	 * @param d_north the relative north translation
	 * @param d_east  the relative east translation
	 */
	static void set_relative_motion(Action *action, float d_north, float d_east) {
		action->north = d_north;
		action->east = d_east;
		action->flags |= (Action::FLAG_NORTH_RELATIVE | Action::FLAG_EAST_RELATIVE);
		action->valid |= (Action::VALID_NORTH | Action::VALID_EAST);
		return;
	}

	/**
	 * set a basic relative North, East, Yaw command that will execute a full rotation upon arrival
	 * @param action  the action to fill
	 * @param d_north the relative north translation
	 * @param d_east  the relative east translation
	 * @param d_yaw   the relative yaw motion
	 */
	static void set_relative_motion(Action *action, float d_north, float d_east, float d_yaw) {
		action->north = d_north;
		action->east = d_east;
		action->yaw = d_yaw;
		action->flags |= (Action::FLAG_NORTH_RELATIVE | Action::FLAG_EAST_RELATIVE | Action::FLAG_YAW_RELATIVE);
		action->valid |= (Action::VALID_NORTH | Action::VALID_EAST | Action::VALID_YAW);
		return;
	}

	/**
	 * just set a relative yaw command.
	 * May no longer be necessary.
	 * @param action the action to fill
	 * @param yaw    the relative yaw motion
	 */
	static void set_yaw_relative(Action *action, float yaw) {
		action->yaw = yaw;
		action->flags |= (Action::FLAG_YAW_RELATIVE);
		action->valid |= (Action::VALID_YAW);
	}

	/**
	 * don't rotate upon arrival at the target location.
	 * @param action the action to fill
	 */
	static void set_no_rotation(Action *action) {
		action->rotation_angle = 0.0;
		action->valid |= (Action::VALID_ROTATION_ANGLE);
	}

	/**
	 * set a basic action command (used by the fixed planner).
	 * @param action   the action to fill
	 * @param d_north  the relative north translation
	 * @param d_east   the relative east translation
	 * @param yaw      the absolute yaw angle for arrival
	 * @param altitude the absolute (AMSL) altitude
	 */
	static void set_basic_motion(Action *action, float d_north, float d_east, float yaw, float altitude) {
		set_relative_motion(action, d_north, d_east);
		action->yaw = yaw;
		action->flags |= Action::FLAG_YAW_ABSOLUTE;
		action->valid |= Action::VALID_YAW;

		action->altitude = altitude;
		action->flags |= Action::FLAG_ALTITUDE_ABSOLUTE;
		action->valid |= Action::VALID_ALTITUDE;
			
	}

};

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
	 * @return  the action to be executed, see the Action class for details
	 */
	virtual Action action() {};

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
