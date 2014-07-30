#pragma once

#include "Instance.h"

#include <vector>
#include <thread>
#include <tuple>
#include <chrono>


class Tours;

/**
This heuristic generates a solution by deviding the x-axis into separete regions.
Every vehicle starts in its own section. Later neigboring sections are merged.
It might be faster to run only the two initial calculations 
instead of every possible devision
**/
class SeparationHeuristic{

	//Switch to enable debug information. 
	static constexpr bool debug_ = false;

	public:
		Tours operator()(const Instance& inst) const;
		Tours operator()(const Instance& inst, std::vector<int>& pos,
					     double bound=std::numeric_limits<double>::infinity()) const;
		SeparationHeuristic(){}

        //optional settings:
        void set_verbosity(int v){verbosity_ = v;}

        void only_initial(bool o){stop_after_initial_ = o;}
        void use_insertion(int i){insertion_heur_ = i;}

	private:

	    int verbosity_  =  0;
	    mutable uint safety_distance_ = 0;
	    bool stop_after_initial_ = false;
	    bool insertion_heur_ = false;


};
