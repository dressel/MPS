#include <vector>
#include <cmath>
#include <string>

using std::vector;
using std::pair;
using std::string;

/* converts vector of gains to corresponding column indices in obs matrix */
vector<int> gains2indices(vector<int>& gains);

/* Finds relative theta, and puts into range [0,35] */
int calculate_theta_rel(vector<int>& theta_rel, vector<double>& theta_v, int theta_j);

/* Creates the observation matrix (theta_rel by rssi) */
vector<vector<double> > make_obs_matrix();

int gains2normgain(int dir_gain, int omni_gain);
int gains2normgain(double dir_gain, double omni_gain);

/**
 * Actual methods to get bearing:
 *  MLE
 *  Bayesian
 *  cc_norm (cross-correlation of normalized)
 *  cc (cross-correlation, only after full rotation)
 *  max (angle at max rssi value received)
 */
double get_bearing_mle(vector<double>& theta_v, vector<int>& gains, vector<vector<double> >& obs_matrix);
double get_bearing_mle(vector<double>& theta_v, vector<int>& gains);

double get_bearing_bayes(vector<double>& theta_v, vector<int>& gains, vector<vector<double> >& obs_matrix);

double get_bearing_cc(vector<double> &angles, vector<double> &gains);

double get_bearing_max(vector<double> &angles, vector<double> &gains);
double get_bearing_max3(vector<double> &angles, vector<double> &gains);

pair<vector<double>, vector<double> > get_gains(string filename);

bool validgain(double gain);
