#pragma once

#include <array>
#include <string>
#include <iostream>

/**
All information about an instance are encapsulated here. 
Basic functions like the information about all jobs or in- and output via files are provided. 
The function that generated random jobs can be useful for some algorithmic tests.
**/
class Job{

	private:
		int _num;
		std::array<int,2> _alpha;
		std::array<int,2> _beta;
		
	public:
		Job() = default ;
		Job(int num,int alpha_x,int alpha_y,int beta_x, int beta_y):_num(num),
									_alpha{{alpha_x,alpha_y}},_beta{{beta_x,beta_y}}{};
		
		const std::array<int,2>& get_alpha() const{return _alpha;}							
		const std::array<int,2>& get_beta() const{return _beta;}					
		
		const std::array<int,2>& alpha() const{return _alpha;}							
		const std::array<int,2>& beta() const{return _beta;}
									
		int length() const;
		int delta_x() const;
		int delta_y() const;
		int num() const{return _num;}
		std::string to_string() const;

		bool operator==(const Job& j) const;
	
};

/** Stream operator for convenience. Prints the string representation of a job. **/
inline 
std::ostream& operator <<(std::ostream &os,const Job &j)
{
	os<<j.to_string();
	return os;
}



//distance functions for any type T where '-' and absolute value are definded
template <typename T> 
T delta(T a, T b){ return (a-b)>(b-a)?(a-b):(b-a);}

template <typename T>
T dist_inf(T a1, T a2, T b1, T b2){ return delta<T>(a1,b1)>delta<T>(a2,b2)?delta<T>(a1,b1):delta<T>(a2,b2);}


template <typename T> 
T dist_inf(const std::array<T,2> &a, const std::array<T,2> &b){ return delta<T>(a[0],b[0]) > delta<T>(a[1],b[1])?delta<T>(a[0],b[0]):delta<T>(a[1],b[1]); }   



