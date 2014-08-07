#pragma once
#include <list>
#include <queue>
#include <vector>
#include <limits> 
#include <array>
#include <tuple>

#include "Job.h"
#include "Instance.h"

class Superjob{
	
private:
	std::vector<Job> jobs_;
	double speed_low;
	double speed_high;
	int jobs_length_;
	bool jobs_positive_;
	uint k_;

	int min_x;
	int max_x;   

public:	
	Superjob() = default;

    //copy constr.
    
 /*   Superjob( const Superjob& other ):
    	speed_low(other.speed_low),
    	speed_high(other.speed_high),
    	jobs_length_(other.jobs_length_),
    	jobs_positive_(other.jobs_positive_),
    	k_(other.k_),
    	min_x(other.min_x),
    	max_x(other.max_x){
    		jobs_.insert(jobs_.end(),other.jobs_.begin(), other.jobs_.end());
    	};

	Superjob(  Superjob&& other ):
    	speed_low(other.speed_low),
    	speed_high(other.speed_high),
    	jobs_length_(other.jobs_length_),
    	jobs_positive_(other.jobs_positive_),
    	k_(other.k_),
    	min_x(other.min_x),
    	max_x(other.max_x){
    		jobs_ = std::move(other.jobs_);
    	};
*/
	// Superjob& operator=(const Superjob& other){
	// 	Superjob tmp( other );

	// 	std::swap( jobs_, tmp.jobs_ );
	// 	///and so on

 //      	return *this;
	// }

	// Superjob& operator=(Superjob&& other){

	// 	jobs_ = std::move(other.jobs_);
	// 	///and so on
	// 	//NOT DONE
 //      	return *this;
	// }




	Superjob(uint k, double low, double high, bool positive):speed_low(low),
					speed_high(high),jobs_length_(0),jobs_positive_(positive),
					k_(k), 
					min_x( std::numeric_limits<int>::max()), 
					max_x( std::numeric_limits<int>::min()){};		

	//returns false if not possible to insert 
	void add(const Job& j);

	double future_quality(const Job& j) const;
	//remove last added job
	double quality() const{ return jobs_length_/length_();}
	int distance_to(const Superjob& other) const;

	int distance_to_depot(const Instance& inst) const;
	int distance_from_depot(const Instance& inst) const;

	bool empty() const {return jobs_.empty();}

	//return the jobs in s, sorted via distance from the start,
 	//additional, return the assignment of each job, this can be done via BFS
	std::vector<std::tuple<Job,uint>> get_sorted_assignment() const;

	bool is_addable(const Job& job) const;

	//find a good tour, make sure that a depot related superjob is present!
	//add Instance to get depoit positions
	static std::vector<Superjob> best_order(const std::vector<Superjob>& jobs, 
											const Instance& inst); 

private:

	bool cycle_or_long_path_(const std::vector<std::list<uint>> &g,
							std::vector<uint>& in) const; 
	
	inline double length_() const{	return (max_x-min_x)/speed_low;}

	std::vector<std::array<int,2>> end_positions_() const;

	std::vector<std::array<int,2>> start_positions_() const;

	std::vector<std::list<uint>> build_graph_(std::vector<uint> &, 
											const std::vector<Job>& ) const;

	std::vector<const Job*> get_extremal_jobs(bool, std::vector<uint> &, 
						const std::vector<std::list<uint>> & ) const;
};	
