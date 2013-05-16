#include "Job.h"
		
#include <cmath>
#include <cassert>

#include <cstring>

int Job::length() const{ 
	return dist_inf(_alpha, _beta);
}


int Job::delta_x() const{
	return delta<int>(_alpha[0],_beta[0]);
}

int Job::delta_y() const{
		return delta<int>(_alpha[1],_beta[1]);
}		

std::string Job::to_string() const{
	return "["	+ std::to_string(_num)+"]: ("
				+ std::to_string(_alpha[0])+"; "+std::to_string(_alpha[1])+") -> "
				+ "(" +std::to_string(_beta[0])+"; "+std::to_string(_beta[1])+")";
}

bool Job::operator==(const Job& j) const{

	return (_num == j._num and _alpha[0]==j._alpha[0] &&  _alpha[1]==j._alpha[1]
                           and _beta[0]==j._beta[0] &&  _beta[1]==j._beta[1] );

}
