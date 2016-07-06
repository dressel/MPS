#include <iostream> //temp
#include "bearingonly.h"

BearingOnly::BearingOnly()
{
	this->noise_sigma = 10;
}

BearingOnly::BearingOnly(double ns)
{
	this->noise_sigma = ns;
}

BearingOnly::~BearingOnly()
{
}
int BearingOnly::type()
{
	return 0;
}
