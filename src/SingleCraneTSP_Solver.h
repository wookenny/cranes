#pragma once

#include "Instance.h"
#include <vector>
#include <fstream>
#include <functional>
#include <tuple>

class Tours;

/**
This class encapsulates the a TSP solution found by a call of "Concorde".
The problem has to be modeled as a symmetric TSP. It replaces
every job by 3 vertices. A start and an end vertex to set correct distances
to all other vertices and a vertex in the middle to ensure that
the itme gets transported, once the initial vertex is visited.

Start vertices are connected with end vertice and vice versa.
In the end, we need to select the right order to traverese the edges.


The TSP format file looks like this:
----------------------------
NAME : 2DVS Instance
TYPE : TSP
DIMENSION: 5
EDGE_WEIGHT_TYPE : EXPLICIT
EDGE_WEIGHT_FORMT :  FULL_MATRIX
EDGE_WEIGHT_SECTION :
0 2 3 3 4
2 0 3 4 2
3 3 0 3 5 
3 4 3 0 2 
4 2 5 2 0
EOF
-------------------------
**/
class SingleCraneTSP_Solver{

	public:
		std::tuple<double, Tours> operator()(const Instance& inst) const;
		
		SingleCraneTSP_Solver();		
	private:       
        void create_TSP_file(const std::vector<std::vector<int>> &) const;
        void write_TSP_file(std::fstream &, const std::vector<std::vector<int>> &) const;
        void set_distances(std::vector<std::vector<int>> &dist, const Instance& i) const;
        std::vector<std::vector<int>>  parse_solution()const;
        double build_tour(Tours&, const std::vector<int>&,const Instance&, 
                                int vehicle,double time_offset=0) const;
	    	
	    		
        mutable uint N;
        mutable uint K;    
        std::function<int (int)> depot_start;
        std::function<int (int)> depot_end;
        std::function<int (int)> job_start;
        std::function<int (int)> job_middle;
        std::function<int (int)> job_end;
        bool is_job_start(int pos)const {return static_cast<uint>(pos)>=2*K and
                                            (static_cast<uint>(pos)-2*K)%3==0;}        
};
