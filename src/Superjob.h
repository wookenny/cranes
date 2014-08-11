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

	static const uint MAX_JOBS = 10;

public:	
	Superjob() = default;

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

	int min() const { return min_x;}
	int max() const { return max_x;}
	double low() const {return speed_low;}
	double high() const {return speed_high;}

	bool is_positive() const { return jobs_positive_;}
	bool is_negative() const { return not jobs_positive_;}
	
	bool empty() const {return jobs_.empty();}

	uint num_vehicles() const{ return k_;}
	uint num_jobs() const{ return jobs_.size();}

	std::vector<Job> copy_jobs() const{
		std::vector<Job> jobs;
		jobs.insert(jobs.begin(),jobs_.begin(),jobs_.end());
		return jobs;
	}

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
