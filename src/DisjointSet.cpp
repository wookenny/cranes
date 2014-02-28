#include "./DisjointSet.h"
#include <cassert>
#include <iostream>

DisjointSet::DisjointSet(unsigned int n):size_(n) {
        // every note is its parent
        for (unsigned int i = 0; i < n; ++i)
            parent_.push_back(i);
        // every node has height 0 and is a root
        rank_ = std::vector<unsigned int>(n, 0);
}

DisjointSet::DisjointSet(const std::vector<int> &permutation) {
    // set to useful initial values
    unsigned int numElements = permutation.size();
    size_ = 0;
    rank_ = std::vector<unsigned int>(numElements, 0);
    parent_ = std::vector<unsigned int>(numElements, numElements);

    for (unsigned int i = 0; i < permutation.size(); ++i) {
        // found set already because numElements=n and parent_[i] <n
        if(parent_.at(i) < numElements )
            continue;
        // found a new set
        ++size_;
        parent_.at(i) = i; // set parent so itself
        rank_.at(i) = 0;  // rank is 0
        unsigned int succ = permutation.at(i);
        while(succ != i  ){// run through the whole subtour
            assert(parent_.at(succ) == numElements );
            parent_.at(succ) = i;// set root
            rank_.at(succ) = 1;// one step from this node to the root
			succ = permutation.at(succ); //increment
		}	
	}
}
		
unsigned int DisjointSet::findSet(unsigned int node){ 
    assert(parent_.size() > node and "Parameter node is to big to be in this DS");
    // notice that each findSet operation sets the parent pointer of each node directly to their root via recursion 
    if( node != parent_.at(node) )
        parent_.at(node) = findSet(parent_.at(node));
    return parent_.at(node);	
}
		
void DisjointSet::linkSets_(unsigned int set1, unsigned int set2){
    // determine which set has a lower path to its root
    if (rank_.at(set1) > rank_.at(set2) )
        parent_.at(set2) = set1;
    else{
        parent_.at(set1) = set2;
        if(rank_.at(set1) == rank_.at(set2) )
            rank_.at(set2) = rank_.at(set2) + 1;
    }
}		 


//------ Here are some GTests for this class---//
#ifdef GTESTS_ENABLED
#include <gtest/gtest.h>
TEST(DisjointSet_Tests, GeneralTests) { 

    auto D1 = DisjointSet{};
    auto D2 = DisjointSet(10);
    const std::vector<int> vec = {0,1,3,4,2};
    auto D3 = DisjointSet(vec);
    EXPECT_EQ(0, D1.size());
    EXPECT_EQ(10, D2.size());
    EXPECT_EQ(5-2, D3.size());

    for(uint i=0; i<D2.size();++i)
        EXPECT_EQ(D2.findSet(i),i);
    D2.unionSets(0,3); D2.unionSets(0,2);
    EXPECT_EQ(D2.findSet(0),D2.findSet(3));
    EXPECT_EQ(D2.findSet(0),D2.findSet(2));
    EXPECT_EQ(D2.findSet(2),D2.findSet(3));
    EXPECT_EQ(1, D2.findSet(1));
    EXPECT_EQ(10-2, D2.size());
    for(uint i=4; i<D2.size()-1;++i)
        EXPECT_EQ(D2.findSet(i),i);
    
    EXPECT_EQ(0, D3.findSet(0));
    EXPECT_EQ(1, D3.findSet(1));
    EXPECT_EQ(D3.findSet(2),D3.findSet(3));
    EXPECT_EQ(D3.findSet(3),D3.findSet(4));
    
}


#else
#endif
