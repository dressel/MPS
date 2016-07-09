#include "fixed_planner2.h"

using std::endl;


FixedPlanner2::FixedPlanner2(string paramfile, string logpath)
	:MyPlanner(paramfile, logpath)
{
	action_index = 0;
	num_actions = 0;
}

int FixedPlanner2::initialize()
{
	/* create the logging file */
	if (start_log()) return -1;

	/* read the configuration file */
	string path = read_config(_param_file);
	if (path == "error") return -1;

	string non_obs_file = path + "fixed.csv";
	if (initialize_actions(non_obs_file)) return -1;

	/* log some stuff and exit without error */
	fprintf(_plannerlog, "FixedPlanner2 initialized.\n###########\n");
	fflush(_plannerlog);
	return 0;
}

int FixedPlanner2::initialize_actions(string non_obs_path)
{
	//vector<vector<double> > actions;
	ifstream action_stream;
	string line;
	action_stream.open(non_obs_path);
	if (!action_stream.is_open())
	{
		fprintf(_plannerlog, "Failed to find action file.\n");
		fflush(_plannerlog);
		return -1;
	}
	string sub;
	int current_size = 0;
	while( !getline(action_stream, line).eof() )
	{
		stringstream ss(line);

		if (line.substr(0,1) != "#")	// line is not a comment
		{
			// file shall be written in north, east, yaw locations
			actions.resize(current_size + 1);
			actions[current_size].resize(3);
			getline(ss, sub, ',');
			actions[current_size][0] = stod(sub);
			getline(ss, sub, ',');
			actions[current_size][1] = stod(sub);
			getline(ss, sub, ',');
			actions[current_size][2] = stod(sub);

			current_size += 1;
		}
	}

	num_actions = current_size;

	return 0;
}


/**
 * Loops through all possible actions, selecting the best.
 */
Action FixedPlanner2::get_action()
{
	//set index and stuff
	int num_actions = actions.size();
	double dnorth = actions[action_index][0] - _uav.y;
	double deast = actions[action_index][1] - _uav.x;
	//double dyaw = actions[action_index][2] - _uav.heading;
	double dyaw = 0.0;

	// update the index
	action_index++;
	if (action_index == num_actions) action_index = 0;

	/* return the action */
	Action action{};
	Action::set_relative_motion(&action, dnorth, deast);
	return action;
}
