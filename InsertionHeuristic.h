#pragma once

#include "Instance.h"

#include <vector>
#include <thread>
#include <tuple>


typedef std::tuple<int,int> interval;
class Tours;

/**
This class provides an heuriostic algorithm to find a solution for the 2DV problem.
It generates a permutation and a vehicle assigment to find the optimal solution with these.
It can be used as a local search for better results.
Probably, a GA would be helpful in this context.
**/
class InsertionHeuristic{

	public:
		Tours operator()(const Instance& inst, const std::vector<uint>&, 
											   const std::vector<uint>&) const;
		Tours operator()(const Instance& inst) const;
		
		InsertionHeuristic():InsertionHeuristic(false){}
		InsertionHeuristic(bool ls):local_search_(ls){}
		
	private:
		bool local_search_;
		uint earliest_startingtime_(const Instance&, const Tours&, const Job&, uint v) const;
		
		
		bool get_best_neighbour(const Instance& inst, const std::vector<uint>&, 
								const std::vector<uint>&, Tours& t) const;
		void insertion_helper(const Instance& inst, const std::vector<uint> &, 
					const std::vector<uint> &assign, Tours &tour) const;
	
		
		inline void intervalsForLeftCone(const scheduledJob& , 
							const Job& , std::vector<interval>&) const;
		inline void intervalsForRightCone(const scheduledJob& , 
							const Job& , std::vector<interval>&) const;
		
		//Switch to enable debug inforamtion. 
		//If it is false, code can be removed during compilation.
		static constexpr bool debug_ = false;	
};
