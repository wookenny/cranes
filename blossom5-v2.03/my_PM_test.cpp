#include <stdio.h>
#include <set>
#include <iostream>

#include "PerfectMatching.h"

/*
Compile like this:
cd SRC
make
cd ..
ar rvs PM.a SRC/*.o SRC/GEOM/*.o SRC/MinCost/*.o
g++ -I./INCLUDE my_PM_test.cpp PM.a -lrt -o PM_test
*/


//this is how I would calculate a PM

int main(){
	
	
	//init my example
	int node_num = 4;
	int edge_num = node_num*(node_num-1);

	
	PerfectMatching pm(node_num, edge_num);
	for (int i=0; i< node_num; ++i) 
		for (int j=0; j< node_num; ++j){ 
			if(i==j) continue;
			pm.AddEdge(i,j,10*rand()+1);
	}	

	pm.Solve();
	//NodeId GetMatch(NodeId i); // alternative way to get the result
	std::set<int> nodes_checked; 
	for (int i=0; i< node_num; ++i){
		if( ! (nodes_checked.find(i)==nodes_checked.end()) )
			continue;//already matched
		int j = pm. GetMatch(i);
		nodes_checked.insert(i); nodes_checked.insert(j);
		std::cout<< i<<"-"<<j<<std::endl;
	}
}


