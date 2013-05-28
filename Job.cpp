#include "Job.h"
		
#include <cmath>
#include <cassert>

#include <cstring>

using namespace std;

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

 
 int point_ordering__(double x1, double t1,
 					  double x2, double t2){
 	//-1: 1 left of 2				  
 	if(x1 < x2 and (x2-x1 > abs(t1-t2)) )
 		return -1;
 	
 	//1: 1 right of 2		
 	if(x1 > x2 and (x1-x2 > abs(t1-t2)) )
 		return 1;
 		
 	//0: no relation		  
 	return 0;				  
 }
 
 
 int Job::getOrdering(const std::tuple<const Job*, double>& job1,
 					  const std::tuple<const Job*, double>& job2) 
{
	const Job* j1 = std::get<0>(job1);
	double t1 = std::get<1>(job1);
	const Job* j2 = std::get<0>(job2);
	double t2 = std::get<1>(job2);
	

	//compare job1.alpha, job2.alpha
	int c1 = point_ordering__( 	j1->alpha()[0], t1, 
								j2->alpha()[0], t2);
	
	//compare job1.alpha, job2.beta
	int c2 = point_ordering__(	j1->alpha()[0], t1, 
								j2->beta()[0] , t2+j2->length());
	
	//compare job1.beta, job2.alpha
	int c3 = point_ordering__(	j1->beta()[0] , t1+j1->length(), 
								j2->alpha()[0], t2);
	
	//compare job1.beta, job2.beta
	int c4 = point_ordering__(	j1->beta()[0] , t1+j1->length(), 
								j2->beta()[0] , t2+j2->length());
 	
 	int max = c1;
	int min = c2;
	
	if(c2>max) max = c2;
	if(c3>max) max = c3;
	if(c4>max) max = c4;
	
	if(c2<min) min = c2;
	if(c3<min) min = c3;
	if(c4<min) min = c4;
	
	//all equal, than that is the right answer
	if(min==max) return min;
	
	//differnce is 1, than it is 1 or -1
	if( 1 == max-min){
		if(0==max){ 
			return -1;
		}else{
			assert(1==max);
			return 1;
		}
	}
	
	
	assert(max-min==2);			
	/*
	cerr<<*j1<<"@"<<t1<<endl;
	cerr<<*j2<<"@"<<t2<<endl;
	cerr<<"c1 "<<c1<<endl;
	cerr<<"c2 "<<c2<<endl;
	cerr<<"c3 "<<c3<<endl;
	cerr<<"c4 "<<c4<<endl;		
 	*/
 	return -2;									
}


