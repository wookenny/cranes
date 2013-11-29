#pragma once

#include "Instance.h"

#include <vector>
#include <thread>
#include <tuple>
#include <chrono>

  


typedef std::tuple<int,int> interval;
class Tours;

/**
This class provides an heuriostic algorithm to find a solution for the 2DV problem.
It generates a permutation and a vehicle assigment to find the optimal solution with these.
It can be used as a local search for better results.
Probably, a GA would be helpful in this context.

There is a second mode, where no assignment is generated.
Every job added to te vehicle, which can execute it te earliest.
**/
class InsertionHeuristic{

	public:
		Tours operator()(const Instance& inst, const std::vector<uint>&, 
											   const std::vector<uint>&) const;
		Tours operator()(const Instance& inst, const std::vector<uint>&) const;
		Tours operator()(const Instance& inst) const;
		
		InsertionHeuristic():InsertionHeuristic(false){}
		InsertionHeuristic(bool ls):local_search_(ls),runs_(1),threads_(-1){}
		void set_runs(uint r){runs_ = r;}
		void set_num_threads(int t){threads_ = t;}

        //optional settings:
        void set_timelimit(int t){time_limit_=t;}
        void set_use_assignment(bool b){no_assignment = not b;}
        void set_verbosity(int v){verbosity_ = v;}
        void set_stop_at_better(bool s){stop_at_better_=s;}
		
	private:
	    bool no_assignment = false;
	    int time_limit_ = -1;
	    int verbosity_  =  0;
	    mutable std::chrono::time_point<std::chrono::system_clock> starting_time_;
		mutable bool local_search_;
		uint runs_;
		int threads_;
		bool stop_at_better_ = false;
		uint earliest_startingtime_(const Instance&, const Tours&, const Job&, uint v) const;
		
		
		bool get_better_neighbour_parallel(const Instance& , std::vector<uint>&, 
								std::vector<uint>&, Tours& t) const;						
		void insertion_helper(const Instance& inst, const std::vector<uint> &, 
					const std::vector<uint> &assign, Tours &tour) const;
        void insertion_helper(const Instance& inst, const std::vector<uint> &, 			
                                           Tours &tour) const;
                                           
		inline void intervalsForLeftCone(const scheduledJob& , 
							const Job& , std::vector<interval>&) const;
		inline void intervalsForRightCone(const scheduledJob& , 
							const Job& , std::vector<interval>&) const;
		
		bool is_permutation(std::vector<uint> v) const{
            sort(begin(v),end(v));
            for(uint i=0; i<v.size();++i)
                if(v[i]!=i)
                    return false;
            return true; 
        }
		
		bool time_remaining() const;
		//Switch to enable debug information. 
		//If it is false, code can be removed during compilation.
		static constexpr bool debug_ = false;
		static constexpr uint random_starts_ = 0;	
};
