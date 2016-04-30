/**
 * bearing_cc.cpp
 */

#include "bearing.h"
#include <fstream>

using std::pair;
using std::string;

/* helper functions */
pair<vector<double>, vector<int> > get_norm_gains(string filename);
double mean(vector<double>& vec);
double std_dev(vector<double>& vec, double mean);
vector<double> normalize(vector<double>& gains);
double cc_helper(vector<double> & angles, vector<double> & gains, vector<double> & ref_angles, vector<double> & ref_gains, double shift);
double cross_correlate(vector<double> & angles, vector<double> & gains, vector<double> & ref_angles, vector<double> & ref_gains);
double interp_gain(double angle, double shift, vector<double> & ref_angles, vector<double> & ref_gains);


double get_bearing_cc(vector<double> & angles, vector<double> & gains)
{
	pair<vector<double>, vector<double> > angles_gains = get_gains("mean_cc.csv");
	vector<double> ref_angles = angles_gains.first;
	vector<double> ref_gains = angles_gains.second;

	//normalize the input gains
	vector<double> norm_gains = normalize(gains);

	// Cross-correlate
	return cross_correlate(angles, norm_gains, ref_angles, ref_gains);
}


pair<vector<double>, vector<int> > get_norm_gains(string filename)
{

	pair<vector<double>, vector<double> > angles_gains = get_gains(filename);
	vector<double> angles = angles_gains.first;
	vector<double> gains = angles_gains.second;

	int vec_length = gains.size();
	vector<int> norm_gains(vec_length);
	int i;
	for (i = 0; i < vec_length; i++)
	{
		norm_gains[i] = (int)(gains[i]);
	}

	return pair<vector<double>, vector<int> >(angles, norm_gains);
}

/**
 * Returns the mean of an input vector
 */
double mean(vector<double>& gains)
{
	int len = gains.size();
	int i = 0;
	double mean = 0.0;
	for (i=0;i<len;i++)
	{
		mean += gains[i];
	}
	mean = mean/len;
	return mean;
}

/**
 * Returns the standard deviation of an input vector
 * Takes in an input vector (gains) and the mean of that vector
 */
double std_dev(vector<double>& gains, double mean)
{
	int len = gains.size();
	int i = 0;
	double sigma_squared = 0.0;
	for (i=0;i<len;i++)
	{
		sigma_squared += pow(gains[i] - mean, 2.0);
	}
	sigma_squared = sigma_squared/(len - 1.0);
	return sqrt(sigma_squared);
}


/**
 * Returns a new, normalized vector
 * Input is the vector of measured gains
 */
vector<double> normalize(vector<double>& gains)
{
	// need to get the mean and std from a copy of gains with no invalid gains
	vector<double> temp_gains;
	temp_gains.clear();

	int len = gains.size();
	for (int i =0; i < len; i++) {
		if (validgain(gains[i])) {
			temp_gains.push_back(gains[i]);
		}
	}
		
	double mg = mean(temp_gains);
	double sd = std_dev(temp_gains, mg);
	
	vector<double> normvec(len);
	int i = 0;
	for (i = 0; i < len; i++)
	{
		if (validgain(gains[i])) {
			normvec[i] = ((gains[i] - mg) / sd);
		} else {
			normvec[i] = gains[i];
		}
	}

	return normvec;
}

/**
 * Inner loop of cross-correlation
 * angles and gains must be same size
 * Ref angles and ref gains must be same size
 */
double cc_helper(vector<double> & angles, vector<double> & gains, vector<double> & ref_angles, vector<double> & ref_gains, double shift)
{
	double c = 0.0;
	int len = angles.size();
	int i = 0;

	for (i = 0; i < len; i++)
	{
		if (validgain(gains[i])) {
			c += gains[i] * interp_gain(angles[i], shift, ref_angles, ref_gains);
		}
	}

	return c;
}


double cross_correlate(vector<double> & angles, vector<double> & gains, vector<double> & ref_angles, vector<double> & ref_gains)
{
	double i = 0.0;
	double cval = 0.0;
	double max_cval = -100000;
	double max_i = 0.0;

	for (i = 0.0; i < 360.0; i+=1.0)
	{
		cval = cc_helper(angles, gains, ref_angles, ref_gains, i);
		if (cval > max_cval)
		{
			max_cval = cval;
			max_i = i;
		}
	}

	return 360.0 - max_i;
}

double interp_gain(double angle, double shift, vector<double> & ref_angles, vector<double> & ref_gains)
{
	// Add the shift to the desired angle, be sure to mod it
	angle = fmod((angle + shift), 360.0);

	// Find the two ref_angles this is between
	int ref_len = ref_angles.size();
	int i = 0;
	// final i value will be between i, i+1
	while (i < (ref_len - 1) && angle > ref_angles[i+1] )
	{
		i++;
	}

	double val;
	//if i == (ref_len - 1), between last and first
	if (i == (ref_len-1))
	{
		val = ((360.0 - angle) * ref_gains[i] );
		val += ((angle - ref_angles[i]) * ref_gains[0]);
		val = val / (360.0 - ref_angles[i]);
	}
	else
	{
		val = ((ref_angles[i+1] - angle) * ref_gains[i] );
		val += ((angle - ref_angles[i]) * ref_gains[i+1]);
		val = val / (ref_angles[i+1] - ref_angles[i]);
	}
	//cout << "here3: " << val << "\n";
	return val;
}
