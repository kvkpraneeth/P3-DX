#include "control/smc.h"

smc::smc(double k1_, double k2_) : k1(k1_), k2(k2_){};

std::vector<double>
smc::getControlSignal()
{
	std::vector<double> temp;
	temp.resize(2);	
	

	return temp;
}
