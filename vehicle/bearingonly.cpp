#include <iostream> //temp
#include "bearingonly.h"

BearingOnly::BearingOnly()
{
	this->noise = 10;
}
BearingOnly::BearingOnly(double ns)
{
	this->noise = ns;
}

int BearingOnly::type()
{
	return 0;
}
