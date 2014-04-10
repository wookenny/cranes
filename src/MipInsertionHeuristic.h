#pragma once

#include "Instance.h"

#include <vector>
#include <thread>
#include <chrono>
																																																																																																																																																																														

class Tours;

/**/
class MipInsertionHeuristic{

	public:
		Tours operator()(const Instance& inst, const std::vector<uint>&) const;
		Tours operator()(const Instance& inst) const;
		
		MipInsertionHeuristic():MipInsertionHeuristic(false){}
		MipInsertionHeuristic(bool ls):local_search_(ls),runs_(1){}
		void set_runs(uint r){runs_ = r;}

        //optional settings:
        void set_timelimit(int t){time_limit_ = t;}
        void set_verbosity(int v){verbosity_ = v;}
		void set_seed(uint s){seed_ = s;}
		void set_cluster_size(uint c){cluster_size_ = c;}	
		void set_debug(bool d){debug_ = d;}

	private:
		bool debug_ = false;
		uint cluster_size_ = 5;
	    uint seed_;
	    int time_limit_ = -1;
	    int verbosity_  =  0;

	    mutable std::chrono::time_point<std::chrono::system_clock>
    														starting_time_;
		mutable bool local_search_;
		uint runs_;
		int threads_;
		
		bool get_better_neighbour(const Instance& , 
									std::vector<uint>&																						, Tours& t) const;						
		
		void insertion_helper(const Instance& inst, const std::vector<uint> &, 			
                                           Tours &tour) const;
};
