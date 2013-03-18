#include "DisjointSet.h"
#include <cassert>
#include <iostream>

DisjointSet::DisjointSet( unsigned int n ):size_(n){
		//every note is its parent
		for(unsigned int i = 0; i<n; ++i)
			parent_.push_back(i);
		//every node has height 0 and is a root
		rank_ = std::vector<unsigned int>( n, 0);
}
		
DisjointSet::DisjointSet( const std::vector<int> &permutation  ){
	//set to useful initial values
	unsigned int numElements = permutation.size();
	size_ = 0;
	rank_   	= std::vector<unsigned int>( numElements, 0);
	parent_ 	= std::vector<unsigned int>( numElements, numElements);

	for(unsigned int i = 0; i<permutation.size(); ++i){
		//found set already because numElements=n and parent_[i] <n
		if( parent_.at(i) < numElements )
			continue;
		//found a new set
		++size_;
		parent_.at(i) = i; //set parent so itself
		rank_.at(i) = 0;  //rank is 0
		unsigned int succ = permutation.at(i);
		while( succ != i  ){//run through the whole subtour
			assert( parent_.at(succ) == numElements );
			parent_.at(succ) = i;//set root
			rank_.at(succ) = 1;//one step from this node to the root
			succ = permutation.at(succ); //increment
		}	
	}
}
		
unsigned int DisjointSet::findSet( unsigned int node){ 
	//notice that each findSet operation sets the parent pointer of each node directly to their root via recursion 
	if( node != parent_.at(node) )
		parent_.at(node) = findSet(parent_.at(node));
	return parent_.at(node);	
}
		
void DisjointSet::linkSets_(unsigned int set1, unsigned int set2){
	//determine which set has a lower path to its root
	if ( rank_.at(set1) > rank_.at(set2) )
		parent_.at(set2) = set1;
	else{
		parent_.at(set1) = set2;
		if( rank_.at(set1) == rank_.at(set2) )
			rank_.at(set2) = rank_.at(set2) + 1;
	}
	
}		 
