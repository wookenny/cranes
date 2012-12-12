cmake_minimum_required(VERSION 2.6)

#project name
project(PerfectMatching-Lib)

SET(PM_Sources
misc.cpp
PMduals.cpp
PMexpand.cpp
PMinit.cpp
PMinterface.cpp
PMmain.cpp
PMrepair.cpp
PMshrink.cpp
GEOM/GPMinit.cpp
GEOM/GPMinterface.cpp
GEOM/GPMkdtree.cpp
GEOM/GPMmain.cpp
MinCost/MinCost.cpp
)


set(CMAKE_CXX_FLAGS "-g -Wextra -O3 -m64 -std=c++0x")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-g -lrt")

ADD_LIBRARY( PM STATIC
${PM_Sources}
)

