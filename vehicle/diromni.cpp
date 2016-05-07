#include <iostream> //temp
#include "diromni.h"

DirOmni::DirOmni()
{
	//this->noise = 10;
}

int DirOmni::type()
{
	return 1;
}

// returns false if ok, true if errror
bool DirOmni::set_means(string mean_path)
{
	string line;

	/* get the mean values */
	ifstream mean_stream;
	mean_stream.open(mean_path);
	if (!mean_stream.is_open())
	{
		cout << "failure withing opening file" << endl;
		return true;
	}

	int i = 0;
	_means.resize(360);
	while (!getline(mean_stream, line).eof())
	{
		int ind = line.find_last_of(",")+1;
		 _means[i] = stod(line.substr(ind, line.length()-ind));
		 i++;
	}
	return false;
}

// TODO: make a better set_stds
bool DirOmni::set_stds(string stds_path)
{
	string line;

	/* get the mean values */
	ifstream stds_stream;
	stds_stream.open(stds_path);
	if (!stds_stream.is_open())
		return true;

	int i = 0;
	_stds.resize(360);
	while (!getline(stds_stream, line).eof())
	{
		 _stds[i] = 1.0;
		 i++;
	}
	return false;
}
