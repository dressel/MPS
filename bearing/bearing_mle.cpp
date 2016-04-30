#include "bearing.h"

#include <iostream>
using std::cout;

/**
 * Returns bearing given vehicle headings and NORMALIZED gains.
 * Just calls the more complex version that requires obs_matrix.
 * This version creates the observation matrix.
 * Because this requires more resources, it might be better to create
 *  the obs_matrix once rather than every time this is created.
 * The obs_matrix is static; it is a model of the system and doesn't change.
 *
 * theta_v is a vector of vehicle headings
 * gains is a vector of the rssi values measured at the vehicle headings
 */
double get_bearing_mle(vector<double>& theta_v, vector<int>& gains)
{
	vector<vector<double> > obs_matrix = make_obs_matrix();
	return get_bearing_mle(theta_v, gains, obs_matrix);
}

/**
 * Returns bearing given vehicle headings and NORMALIZED gains.
 *
 * theta_v is a vector of vehicle headings
 * gains is a vector of the normalized gain values measured at the headings
 */
double get_bearing_mle(vector<double>& theta_v, vector<int>& gains, vector<vector<double> >& obs_matrix)
{

	/* create vector for theta_rel */
	int vec_length = theta_v.size();
	vector<int> theta_rel(vec_length);


	/* convert gains to indices */
	vector<int>indices = gains2indices(gains);

	/* Loop over all possible jammer locations */
	int i;
	int theta_j;
	double prob;
	double best_prob = 0.0;
	int best_theta_j = -1;
	double oprob;
	//int test_val = 68;
	//cout << "test value = " << test_val << "\n";
	for (theta_j = 0; theta_j < 360; theta_j++)
	{
		/* for each theta_j, compute vector of theta_rel values */
		calculate_theta_rel(theta_rel, theta_v, theta_j);

		/* now run the mle stuff */
		// loop over all theta_rel
		prob = 1.0;
		for (i = 0; i < vec_length; i++)
		{
			prob = prob * obs_matrix[theta_rel[i]][indices[i]];
		}
		//cout << "theta_j = " << theta_j << ", " << "prob = " << prob << "\n";
		if (prob > best_prob)
		{
			best_prob = prob;
			best_theta_j = theta_j;
		}
	}
	/* The following correction is prob best only for simlations */
	best_theta_j = (best_theta_j + 4) % 360;
	return best_theta_j;
}
