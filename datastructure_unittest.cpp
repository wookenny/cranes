//Compiling with this: 

//g++ unittest.cpp -std=c++0x -lgtest_main -lgtest -lpthread 

#include <gtest/gtest.h>
#include "Job.h"
#include "Tours.h"

TEST(Job_Class/*Testcase Name*/, Lengths_and_differences /*Test name*/) {
	int a1 = 1; int a2 = 2;
	int b1 = -4; int b2 = -2;
	Job j(0,a1,a2,b1,b2);
  	EXPECT_EQ(j.delta_x(), 5);
  	EXPECT_EQ(j.delta_y(), 4);
  	EXPECT_EQ(j.length(), 5);
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
  	//ceck assertions
  	//EXPECT_DEATH(t.add_job(&j4,4,0),"what?");
} 

