cmake_minimum_required(VERSION 2.6)

#project name
project(PerfectMatching-Test)

ADD_SUBDIRECTORY(./SRC)

 # collect all sources (copy from 'ls -1 *.cpp *.h')
SET(PM_Test_Sources
my_PM_test.cpp
)

set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -pedantic -O3 -m64 -std=c++0x -I./INCLUDE ")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-g -lrt ")

#define executable 
ADD_EXECUTABLE(PM_test ${PM_Test_Sources} )
target_link_libraries (PM_test PM) 

