#pragma once

#include "Instance.h"
#include "Superjob.h"

#include <vector>

class Tours;

/**
TODO: Describe procedure!
**/
class ConsolidationHeuristic{

	//Switch to enable debug information. 
	static constexpr bool debug_ = false;

	public:
		Tours operator()(const Instance& inst) const;
		Tours operator()(const Instance& inst, const std::vector<Superjob>&) const;
		ConsolidationHeuristic() = default;

        //optional settings:
        void set_verbosity(int v){verbosity_ = v;}


	private:

	    int verbosity_  =  0;
	    mutable uint safety_distance_ = 0;

	    std::vector<Superjob> greedy_consolidation_(const Instance& inst) const;
		std::vector<Superjob> build_superjobs_(const Instance& inst,
						double low_speed, double high_speed,std::vector<Job>& pos_jobs, 
						std::vector<Job>& neg_jobs) const;
};
