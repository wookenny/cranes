#pragma once

#include <iostream>
#include <array>
#include <vector>
#include <string>

#include "Job.h"
#include "Tours.h"

/**
All information about an instance are encapsulated here. 
Basic functions like the information about all jobs or in- and output 
via files are provided. 
The function that generated random jobs can be useful
 for some algorithmic tests.
**/
class Instance{
	
	private:
		unsigned int num_vehicles_ = 2;
		unsigned int safety_distance_ = 0;
		std::vector< std::array<int, 2> > depotPositions_;
		std::vector<Job> jobs_;
		
		bool debug_ = false;
	public:
		
		//constrs
		Instance() = default;
		explicit Instance(unsigned int i): num_vehicles_(i){}
 		explicit Instance(std::string file);	
		
		
		/** Writes the instance to a 2DVS File with the given name. 
		A given comment will be written at the end.
		No special format for the commentstring needed.**/
		void writeToFile(std::string filename, std::string comments = "") const;
		
		//simple setter, getter and initial.
		std::string to_string() const;
		
		void set_num_vehicles(unsigned int num){num_vehicles_=num;}
		unsigned int num_vehicles() const {return num_vehicles_;}		
		void add_job(const Job& j){jobs_.push_back(j);}
		void add_depotposition(const std::array<int, 2> &d){
			depotPositions_.push_back(d);}
		void debug(bool d){debug_=d;}	
			

		//iterator over jobs to use a range based for loop
		unsigned int num_jobs() const{ return jobs_.size();}
		std::vector<Job>::const_iterator begin() const{return jobs_.begin();}  
		std::vector<Job>::const_iterator end() const{return jobs_.end();}
		const Job& operator[](unsigned int i) const{return jobs_[i];}
		std::array<int, 2> get_depot(unsigned int i) const{
		    assert(i<depotPositions_.size()); 
		    return depotPositions_[i]; }  
		
		//generates n random jobs in the given bounds
		void generate_random_jobs(int n, int min_x, int max_x, 
		                          int min_y, int max_y, unsigned int seed=0);
		//generates depot positions such that it fits to the number of depots
		void generate_random_depots(int min_x, int max_x, int min_y, 
		                            int max_y, unsigned int seed=0);
		
		//get the smallest bounding box around the whole instance
		std::array<int,4> get_bounding_box() const;
		
		//verification for tours
		bool   verify(Tours& t) const;
		double makespan(Tours& t) const;
		
		Tours get_MIP_solution(bool collision_free = false,
		                       bool LP_relax = false, 
		                       bool debug = false) const;
		unsigned int get_upper_bound() const;
		
	private:
		void parse_line_(std::string &line);	
			
	
};

/** Stream operator for convenience. 
Prints the string representation of the instance. **/
inline 
std::ostream& operator <<(std::ostream &os,const Instance &inst)
{
	os<<inst.to_string();
	return os;
}
