#include "my_PM_test.h"

#include <stdio.h>
#include <set>
#include <iostream>

#include "PerfectMatching.h"

using namespace std;

//this is how I would calculate a PM

void run_PM(int n){
	
	
	//init my example
	int node_num = n;
	int edge_num = node_num*(node_num-1);

	
	PerfectMatching pm(node_num, edge_num);
	for (int i=0; i< node_num; ++i) 
		for (int j=0; j< node_num; ++j){ 
			if(i==j) continue;
			pm.AddEdge(i,j,10*rand()+1);
	}	
	cout<< "Build a graph with "<<n<<" vertices and "<<edge_num<<" edges"<<endl;

	pm.Solve();
	//NodeId GetMatch(NodeId i); // alternative way to get the result
	set<int> nodes_checked; 
	for (int i=0; i< node_num; ++i){
		if( ! (nodes_checked.find(i)==nodes_checked.end()) )
			continue;//already matched
		int j = pm.GetMatch(i);
		nodes_checked.insert(i); nodes_checked.insert(j);
		cout<< i<<"-"<<j<<endl;
	}
}


