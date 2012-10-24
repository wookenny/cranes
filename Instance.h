#pragma once

#include "banned.h"
#include <iostream>
#include <array>
#include <vector>
#include <string>

#include "Job.h"
#include "Tours.h"

/**
All informations about an instance are encapsuled here. 
Basic functions like the informations about all jobs or in- and output via files are provided. 
The function that generated random jobs can be usefull for some algorithmic tests.
**/
class Instance{
	
	private:
		unsigned int _num_vehicles;
		std::vector< std::array<int, 2> > _depotPositions;
		std::vector<Job> _jobs;
		
	public:
		
		//constrs
		Instance() = default;
		Instance(unsigned int i): _num_vehicles(i){}
 		Instance(std::string file);	
		
		//simple setter, getter and initial.
		std::string to_string() const;
		
		void set_num_vehicles(unsigned int num){_num_vehicles=num;}
		unsigned int num_vehicles() const {return _num_vehicles;}		
		void add_job(const Job& j){_jobs.push_back(j);}
		void add_depotposition(const std::array<int, 2> &d){
			_depotPositions.push_back(d);}

		//iterator over jobs to use a range based for loop
		unsigned int num_jobs() const{ return _jobs.size();}
		std::vector<Job>::const_iterator begin() const{return _jobs.begin();}  
		std::vector<Job>::const_iterator end() const{return _jobs.end();}
		const Job& operator[](unsigned int i) const{return _jobs[i];}
		const std::array<int, 2> get_depot(unsigned int i) const{return _depotPositions[i]; }  
		
		//generates n random jobs in the given bounds
		void generate_random_jobs(int n, int min_x, int max_x, int min_y, int max_y, unsigned int seed=0);
		//generates depot positions such that it fits to the number of depots
		void generate_random_depots(int min_x, int max_x, int min_y, int max_y, unsigned int seed=0);
		
		//verification for tours
		bool   verify(Tours& t) const;
		double makespan(Tours& t) const;
		
		Tours get_MIP_solution() const;
		unsigned int get_upper_bound() const;
		
	private:
		void _parse_line(std::string &line);	
			
	
};

/** Stream operator for convenience. Prints the string representation of the instance. **/
inline 
std::ostream& operator <<(std::ostream &os,const Instance &inst)
{
	os<<inst.to_string();
	return os;
}
