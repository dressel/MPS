/**
 * @file fixed_planner.h
 *
 * declaration of fixed planner class.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef _FIXED_PLANNER_H_
#define _FIXED_PLANNER_H_

#include <string>

#include "planner.h"


/**
 * @class FixedPlanner
 * 
 * A "planner" that cycles through a set of commands.
 * Commands used are in the file that is passed into it.
 * 
 */
class FixedPlanner : public Planner {

public:

	/**
	 * constructor
	 * @param command_file_name  the path to the command file.
	 */
	FixedPlanner(std::string logfile_dir, const char * command_file_name);

	/**
	 * desctructor
	 */
	~FixedPlanner();

	/**
	 * load in all the commands from file.
	 * @return  true if successfully initialized
	 */
	int initialize();

	/**
	 * return the next command.
	 * @return  the command
	 */
	vector<float> action();

private:

	// the file containing all the commands
	const char * _command_file_name;

	// the logfile to write things to
	FILE *_logfile;

	// the commands themselves
	vector<float> _cmd_north;
	vector<float> _cmd_east;
	vector<float> _cmd_yaw;
	vector<float> _cmd_alt;

	// file details
	int _num_cmds;

	// index param
	int _cmd_index;

};



#endif	/* _FIXED_PLANNER_H_ */
