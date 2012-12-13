#pragma once

#include <vector>

/**
This class is used to handle sets effincently. It supports to determine weather to elements are in the same set.
The second important feature is to join two sets given by two elements out of different sets.
Without further notation, all the values inside the disjoint sets are 1,...,n.
Another possiblility is to use "boost/disjoint_sets.hpp".
**/
class DisjointSet{
	
	private:
		/** the number of the disjoint sets, possible value is between 1 and n**/
		unsigned int _size;

		/** a node that represents the set for each element, this elements changes during some operations**/
		std::vector<unsigned int>  _parent;
		/**The rank is an upper bound on the height of a key. It determines how many steps are needed at most to find the root.**/
		std::vector<unsigned int>  _rank; 

		/**Both sets will regard the same root. This operation shpould one be used with notes that are roots. Do not call it directly. **/
		void linkSets(unsigned int set1, unsigned int set2);	
	public:

		//constr.
		/** Creates a set of n disjoints sets */
		DisjointSet( unsigned int n = 0 );
		/** Creates disjoint sets out of a permutation vector. The permutation defines members of the same set. 
		It's the representation of some subtours.*/
		DisjointSet( const std::vector<int> &permutation  );
	
		//public methods
		/** Returns the number of disjoint sets. */
		unsigned int size() const{ return _size; }
		/** Returns the root node for a given node.*/
		unsigned int findSet( unsigned int node);
		/** Joins togehter two sets given by arbitrary elements of the sets.*/
		void unionSets(unsigned int nodeOutOfSet1, unsigned int nodeOutOfSet2){ 
			linkSets( findSet(nodeOutOfSet1),findSet(nodeOutOfSet2) );
			--_size;
		}
	 

};

