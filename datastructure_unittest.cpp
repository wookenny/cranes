//Compiling with this: 

//g++ unittest.cpp -std=c++0x -lgtest_main -lgtest -lpthread 

#include <gtest/gtest.h>
#include <random>
#include <iostream>

#include "Job.h"
#include "Tours.h"
#include "Instance.h"

TEST(Job_Class/*Testcase Name*/, Lengths_and_differences /*Test name*/) {
	

    std::random_device rnd; // obtain a random number from hardware
    std::mt19937 eng(rnd()); // seed the generator
    std::uniform_int_distribution<> distr(-1000, 1000); // define the range
    for(int i = 10000; i>0; --i){
        int a1 = distr(eng); int a2 = distr(eng);
        int b1 = distr(eng); int b2 = distr(eng);
        int index = abs(distr(eng));
        
        Job j1(index,a1,a2,b1,b2);
        Job j2(index-1,a2,a1,b1,b2);
        EXPECT_EQ(j1.delta_x(), abs(a1-b1));
        EXPECT_EQ(j1.delta_y(), abs(a2-b2));
        EXPECT_EQ(j1.length(), std::max(abs(a1-b1),abs(a2-b2)));
        EXPECT_EQ(j1.num(),index);
        EXPECT_EQ(j1==j1,true);
        EXPECT_EQ(j2==j1,false);
    } 
    
    
    
    
} 


TEST(Tour_Class/*Testcase Name*/, Adding_and_Sorting /*Test name*/) {
	Job j1(1, 1,2,3,4);
	Job j2(2, 2,3,4,5);
 	Job j3(3, 3,4,5,6);
  	Job j4(4, 4,5,6,7);
  	Job j5(5, 4,5,6,7);
  	Tours t(2);
  	t.add_job(&j1,2,0);
  	t.add_job(&j2,1,0);
  	t.add_job(&j3,3,1);
  	t.add_job(&j4,4,0);
  	t.sort_jobs();
  	//contains tests
  	EXPECT_EQ(t.contains(&j1), true);
  	EXPECT_EQ(t.contains(&j2), true);
  	EXPECT_EQ(t.contains(&j3), true);
	EXPECT_EQ(t.contains(&j4), true);
  	EXPECT_EQ(t.contains(&j5), false);
  	//size checks
  	EXPECT_EQ(t.num_jobs(), 4U);
  	EXPECT_EQ(t.num_tours(),2U);
  	EXPECT_EQ(t[0].size(), t.num_jobs(0));
  	EXPECT_EQ(t[0].size(), 3U);
  	
  	EXPECT_EQ(t[1].size(), t.num_jobs(1));
  	EXPECT_EQ(t[1].size(), 1U);
  	//check assertions
    //EXPECT_DEATH(t.add_job(&j4,4,0),"");
    Job j6(6, 4,5,6,7);
    //EXPECT_DEATH(t.add_job(&j6,4,3),"");
    //EXPECT_DEATH(t.add_job(&j6,4,-1),"");
} 


TEST(Tour_Class/*Testcase Name*/, Verifying_Tours /*Test name*/) {
	  
	Instance inst(3);
  	inst.add_depotposition( {{7,2}} );
  	inst.add_depotposition( {{11,2}} );
  	inst.add_depotposition( {{13,2}} );
  	Job j1(1, 4,2,2,1);	  inst.add_job(j1);
	Job j2(2, 6,2,7,2);	  inst.add_job(j2);
 	Job j3(3, 7,1,10,2);  inst.add_job(j3);
  	Job j4(4, 12,3,12,0); inst.add_job(j4);
  	Job j5(5, 11,1,15,1);  inst.add_job(j5);

	//Valid tour
	{
	Tours t(3);
  	t.add_job(&inst[1 -1],28,2);
  	t.add_job(&inst[2 -1],24,2);
  	t.add_job(&inst[3 -1],17,2);
  	t.add_job(&inst[4 -1],1,2);
	t.add_job(&inst[5 -1],5,2); 
	EXPECT_EQ(inst.verify(t),true);
	}
	
	//Valid tour
	{
	Tours t(3);
  	t.add_job(&inst[1 -1],3,0);
  	t.add_job(&inst[2 -1],9,0);
  	t.add_job(&inst[3 -1],4,1);
  	t.add_job(&inst[4 -1],1,2);
	t.add_job(&inst[5 -1],5,2); 
	EXPECT_EQ(inst.verify(t),true);
	}
	
	
	//Invalid tour: too many cranes used
	{
	Tours t(4);
  	t.add_job(&inst[1 -1],3,0);
  	t.add_job(&inst[2 -1],9,0);
  	t.add_job(&inst[3 -1],4,1);
  	t.add_job(&inst[4 -1],1,2);
	t.add_job(&inst[5 -1],5,3); 
	EXPECT_EQ(inst.verify(t),false);
	}
	
	//Invalid tour: not reachable
	{
	Tours t(3);
  	t.add_job(&inst[1 -1],11,0);
  	t.add_job(&inst[2 -1],1,0);
  	t.add_job(&inst[3 -1],3,0);
  	t.add_job(&inst[4 -1],1,1);
	t.add_job(&inst[5 -1],5,1); 
	EXPECT_EQ(inst.verify(t),false);
	}
	
	//Invalid tour: cross tour
	{
	Tours t(3);
  	t.add_job(&inst[1 -1],3,0);
  	t.add_job(&inst[2 -1],10,1);
  	t.add_job(&inst[3 -1],10,0);
  	t.add_job(&inst[4 -1],1,2);
	t.add_job(&inst[5 -1],5,2); 
	EXPECT_EQ(inst.verify(t),false);
	}
	
	//Invalid tour: cross job
	{
	Tours t(3);
  	t.add_job(&inst[1 -1],12,0);
  	t.add_job(&inst[2 -1],1,0);
  	t.add_job(&inst[3 -1],3,0);
  	t.add_job(&inst[4 -1],2,1);
	t.add_job(&inst[5 -1],2,2); 
	EXPECT_EQ(inst.verify(t),false);
	}
	
}


