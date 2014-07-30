#pragma once

#include <tuple>
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
		Job():_num(0),_alpha{{0,0}},_beta{{0,0}}{};
		Job(const Job&)= default;
		Job(const Job* j):_num(j->_num),
		                  _alpha{{j->_alpha[0],j->_alpha[1]}},
		                  _beta{{j->_beta[0],j->_beta[1]}} {};
		Job(int num,int alpha_x,int alpha_y,int beta_x, int beta_y):_num(num),
									_alpha{{alpha_x,alpha_y}},
									_beta{{beta_x,beta_y}}{};
		
		const std::array<int,2>& get_alpha() const{return _alpha;}							
		const std::array<int,2>& get_beta() const{return _beta;}					
		
		const std::array<int,2>& alpha() const{return _alpha;}							
		const std::array<int,2>& beta() const{return _beta;}
									
		int length() const;
		int delta_x() const;
		int delta_y() const;
		int num() const{return _num;}
		std::string to_string() const;

        void shift(int x_shift, int y_shift) {
            _alpha[0] += x_shift; _beta[0] += x_shift;
            _alpha[1] += y_shift; _beta[1] += y_shift;
        }

        bool contained_in_xrange(int x1, int x2)  const{
        	return x1 <= _alpha[0] and _alpha[0] <= x2 and
        		   x1 <=  _beta[0] and  _beta[0] <= x2;
        }

        bool contained_in_yrange(int y1, int y2) const{
        	return y1 <= _alpha[1] and _alpha[1] <= y2 and
        		   y1 <=  _beta[1] and  _beta[1] <= y2;
        }

		bool operator==(const Job& j) const;
        bool operator!=(const Job& j) const{return !operator==(j);}
        
		/*
		This method considers the implication between the
		assignment of two jobs with given startingtimes.
		They can be the requirement to be left/right of each other,
		no requirement at all or simply a state were they must have 
		a different starting time, because we found the right AND the left
		requirement.
		
		returns -1 if first job has to be left of the second.
		returns 1 if first job has to be right of the second.
		returns 0 if no ordering is enforced by these two jobs with the 
		starting times
		returns -2 if these two jobs are not doable with thegiven starting times
		*/
		static int getOrdering(const std::tuple<const Job*, double>&,
							   const std::tuple<const Job*, double>&);
};



/** Stream operator for convenience. Prints the string representation 
of a scheduled job. **/
inline 
std::ostream& operator <<(std::ostream &os, const std::tuple<const Job*, double> &j)
{
	os<< std::get<0>(j)->to_string()<<" @"<<std::get<1>(j);
	return os;
}

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



