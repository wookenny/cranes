#pragma once

#include "Instance.h"

#include <vector>
#include <thread>

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
											   const std::vector<uint>&S) const;
		Tours operator()(const Instance& inst) const;
		
		InsertionHeuristic():InsertionHeuristic(false){}
		InsertionHeuristic(bool ls):local_search_(ls){}
		
	private:
		bool local_search_;
		uint earliest_startingtime_(const Instance&, const Tours&, const Job&, uint v) const;
};
