#include "momdp_planner.h"
#include <numeric>

using std::vector;

AlphaVector::AlphaVector(int a, vector<float> vec)
{
	this->a = a;
	this->vec = vec;
}

MOMDPPlanner::MOMDPPlanner(string paramfile)
{
	string path = read_config(paramfile);

	// technically, we should check, but we really know this is ok
	// perhaps I could do a dynamic cast...
	DF * f = static_cast<DF *>(this->filter);
	this->_n = f->n;
	int n = f->n;
	int n2 = n*n;
	//std::cout << "n = " << n << std::endl;
	policy.resize(n2);
	_x = n/2;
	_y = n/2;

	//std::cout << path << std::endl;
	string a_path = path + "policy/alpha_actions.bin";
	string o_path = path + "policy/alpha_obs.bin";
	string v_path = path + "policy/alpha_vectors.bin";

	ifstream a_file;
	ifstream o_file;
	ifstream v_file;
	a_file.open(a_path);
	o_file.open(o_path);
	v_file.open(v_path);
	int i, a, o;
	float v;
	int s;
	vector<float>* temp;
	while (!a_file.eof())  // convert to while loop
	{
		a_file.read(reinterpret_cast<char*>(&a), sizeof(int));
		o_file.read(reinterpret_cast<char*>(&o), sizeof(int));

		//create a temporary vector
		temp = new vector<float>(n2);
		for (s = 0; s < n2; s++)
		{
			v_file.read(reinterpret_cast<char*>(&v), sizeof(float));
			(*temp)[s] = v;
		}
		policy[o].push_back(AlphaVector(a, *temp)); //TODO new issues
	}
}


vector<float> MOMDPPlanner::action()
{
	double o = get_obs();
	filter->update(x, o);

	/* Convert 2-D belief into 1-D vector */
	DF * f = static_cast<DF *>(this->filter);
	vector<float> b1d (_n*_n);
	int x,y,i;
	i = 0;
	for (x = 0; x < _n; x++)
	{
		for (y = 0; y < _n; y++)
		{
			b1d[i] = f->b[x][y];
			i++;
		}
	}

	/* Determine observed index from vehicle state */
	int obs_ind = _x + _n*_y;

	/* inner_product */
	int max_a, num_vecs;
	double V, max_V;

	vector<float> command(3,0.0);

	//loop
	bool not_rotated = true;
	int dx, dy;
	int old_x = _x;
	int old_y = _y;
	while (not_rotated)
	{
		num_vecs = policy[obs_ind].size();	//alpha vecs for this obs_ind
		max_V = -99999;			// TODO: maybe use int_min or something
		max_a = 0;
		for (i = 0; i < num_vecs; i++)
		{
			V = inner_product(b1d.begin(), b1d.end(), policy[obs_ind][i].vec.begin(), 0);
			if (V > max_V)
			{
				max_V = V;
				max_a = policy[obs_ind][i].a;
			}
		}
		//std::cout << "max_a = " << max_a << std::endl;
		
		// If we rotate, that is cool
		if (max_a > 7);
			not_rotated = false;

		// convert action to change in _x, _y
		// TODO: need to check that we stay in bounds
		if (max_a == 0)
			_y += 1;
		else if (max_a == 1)
		{
			_x++;
			_y++;
		}
		else if (max_a == 2)
			_x++;
		else if (max_a == 3)
		{ //se
			_x++;
			_y--;
		}
		else if (max_a == 4)
			_y--;
		else if (max_a == 5)
		{
			_x--;
			_y--;
		}
		else if (max_a == 6)
			_x--;
		else if (max_a == 7)
		{
			_x--;
			_y++;
		}
	}

	//TODO: set the step_size somewhere
	double step_size = 10.0;
	command[0] = step_size*(_y - old_y);
	command[1] = step_size*(_x - old_x);

	return command;
}