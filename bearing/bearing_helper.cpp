/**
 * bearing_helper.cpp
 *
 * Contains a lot of functions needed for the bearing calculations
 */

#include <fstream>
#include "bearing.h"

/* Most of these are needed for make_obs_matrix */
using std::getline;
using std::stod;
using std::string;

/**
 * Converts two gains to a single gain
 */
int gains2normgain(int dir_gain, int omni_gain)
{
	if (dir_gain > 20) {
		return dir_gain;
	}

	if (omni_gain > 20) {
		return omni_gain;
	}
	return dir_gain - omni_gain + 5;
}

/**
 * Similar version, but in case gains are given as doubles
 */
int gains2normgain(double dir_gain, double omni_gain)
{
	if (dir_gain > 20) {
		return dir_gain;
	}

	if (omni_gain > 20) {
		return omni_gain;
	}
	
	return (int)(dir_gain - omni_gain + 5);
}

/**
 * Creates vector of indices corresponding to vector of normalized gains
 * Assumes we are dealing with observation model w/38 possible observations
 *  and a null observation (for a total of 39)
 * Also assumes minimum is -26, and it is continuous integer range to 11
 *
 * Confirmed working, but might want to tweak invalid ranges
 * As in, is a measurement outside our range null or rounded to nearest?
 */
vector<int> gains2indices(vector<int>& gains)
{
	int min_gain = -26; // this is hard-coded
	int vec_length = gains.size();
	vector<int> indices(vec_length);
	int i;
	for (i = 0; i < vec_length; i++)
	{
		if (gains[i] > 20)
			indices[i] = 38; //null
		else if (gains[i] < -26)
			indices[i] = 0; //too low
		else if (gains[i] > 11)
			indices[i] = 37; //too high
		else
			indices[i] = gains[i] - min_gain; //just right
	}
	return indices;
}


/**
 * Determines the theta_rel vector given vehicle angles and jammer angle
 * The resulting theta_rel is from [0,35]
 * 
 * Confirmed to be working
 */
int calculate_theta_rel(vector<int>& theta_rel, vector<double>& theta_v, int theta_j)
{
	int vec_length = theta_rel.size();
	int temp;

	/* Loop through theta_v */
	int i;
	for (i = 0; i < vec_length; i++)
	{
		/* subtract theta_j from theta_v */
		/* Ensure that this is in [0,359] */
		temp = (int)round(theta_v[i] - theta_j);
		if (temp < 0)
			temp = (temp+360) % 360;

		/* Convert this value to [0,35] */
		theta_rel[i] = (int)(round(temp / 10.0)) % 36;
	}

	return 0;
}


vector<vector<double> > make_obs_matrix(string filename)
{
	/* Create a file object named infile */
	// TODO: handle file not existing
	std::ifstream infile(filename.c_str());

	/* Create the matrix */
	vector<vector<double> > obs_matrix(36, vector<double>(39));
	/* Hard-coded for observation model with 36 rows and 39 columns */
	int row;
	int col;
	string val;
	for (row = 0; row < 36; row++)
	{
		//obs_matrix[row] = vector<double>
		for (col = 0; col < 38; col++)
		{
			getline(infile, val, ',');
			obs_matrix[row][col] = stod(val);
		}
		getline(infile, val);
		obs_matrix[row][38] = stod(val);
	}

	return obs_matrix;
}

vector<vector<double> > make_obs_matrix()
{
	return make_obs_matrix("obs_model.csv");
}

/**
 * Now flexible to different sizes (not just 36). Should work
 * Don't add invalid gains
 * TODO: rename get_valid_gains
 */
pair<vector<double>, vector<double> > get_gains(string filename)
{
	/* Create a file object named infile */
	std::ifstream infile(filename.c_str());

	double angle;
	vector<double> angles;
	double gain;
	vector<double> gains;
	char comma;

	while (infile >> angle >> comma >> gain)
	{
		if (validgain(gain))
		{
			angles.push_back(angle);
			gains.push_back(gain);
		}
	}

	return pair<vector<double>, vector<double> >(angles, gains);
}

bool validgain(double gain)
{
	double _nullobs = 2147483647;
	if (gain == _nullobs)
		return false;
	else
		return true;
}

/*
pair<vector<double>, vector<double> > get_gains(string filename)
{
	// Create a file object named infile
	std::ifstream infile(filename.c_str());

	double angle;
	vector<double> angles(36);
	double gain;
	vector<double> gains(36);
	char comma;

	int i = 0;
	while (infile >> angle >> comma >> gain)
	{
		angles[i] = angle;
		gains[i] = gain;
		i++;
	}

	return pair<vector<double>, vector<double> >(angles, gains);
}
*/
