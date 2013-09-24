#pragma once

#include "Instance.h"
#include <vector>
#include <fstream>

class Tours;

/**
This class encapsulates the a TSP solution found by a call of "Concorde".
The problem has to be modeled as a symmetric TSP. It replaces
every job by 3 vertices. A start and an end vertex to set correct distances
to all other vertices and a vertex in the middle to ensure that
the itme gets transported, once the initial vertex is visited.

Start vertices are conencted with end vertice and vice versa.
In the end, we need tro select the right order to traverese the edges.

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
		Tours operator()(const Instance& inst) const;
				
	private:       
        void create_TSP_file(const std::vector<std::vector<int>> &) const;
        void write_TSP_file(std::fstream &, const std::vector<std::vector<int>> &) const;
        void set_distances(std::vector<std::vector<int>> &dist, const Instance& i) const;
};
