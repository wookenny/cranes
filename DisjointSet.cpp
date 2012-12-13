#include "DisjointSet.h"
#include <cassert>
#include <iostream>

DisjointSet::DisjointSet( unsigned int n ):_size(n){
		//every note is its parent
		for(unsigned int i = 0; i<n; ++i)
			_parent.push_back(i);
		//every node has height 0 and is a root
		_rank = std::vector<unsigned int>( n, 0);
}
		
DisjointSet::DisjointSet( const std::vector<int> &permutation  ){
	//set to useful initial values
	unsigned int numElements = permutation.size();
	_size = 0;
	_rank   	= std::vector<unsigned int>( numElements, 0);
	_parent 	= std::vector<unsigned int>( numElements, numElements);

	for(unsigned int i = 0; i<permutation.size(); ++i){
		//found set already because numElements=n and _parent[i] <n
		if( _parent.at(i) < numElements )
			continue;
		//found a new set
		++_size;
		_parent.at(i) = i; //set parent so itself
		_rank.at(i) = 0;  //rank is 0
		unsigned int succ = permutation.at(i);
		while( succ != i  ){//run through the whole subtour
			assert( _parent.at(succ) == numElements );
			_parent.at(succ) = i;//set root
			_rank.at(succ) = 1;//one step from this node to the root
			succ = permutation.at(succ); //increment
		}	
	}
}
		
unsigned int DisjointSet::findSet( unsigned int node){ 
	//notice that each findSet operation sets the parent pointer of each node directly to their root via recursion 
	if( node != _parent.at(node) )
		_parent.at(node) = findSet(_parent.at(node));
	return _parent.at(node);	
}
		
void DisjointSet::linkSets(unsigned int set1, unsigned int set2){
	//determine which set has a lower path to its root
	if ( _rank.at(set1) > _rank.at(set2) )
		_parent.at(set2) = set1;
	else{
		_parent.at(set1) = set2;
		if( _rank.at(set1) == _rank.at(set2) )
			_rank.at(set2) = _rank.at(set2) + 1;
	}
	
}		 
