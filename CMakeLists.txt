cmake_minimum_required(VERSION 2.6)

#project name
project(2D-VehicleScheduling)

ADD_SUBDIRECTORY(./blossom5-v2.03/SRC)

#some path settings
set(CPLEXDIR "/usr/site-local/ILOG/CPLEX_Studio124/cplex")
set(CPLEXINCDIR "${CPLEXDIR}/include")
set(CPLEXLIBDIR "${CPLEXDIR}/lib/x86-64_sles10_4.1/static_pic")

set(CONCERTDIR "/usr/site-local/ILOG/CPLEX_Studio124/concert")
set(CONCERTINCDIR "${CONCERTDIR}/include")
set(CONCERTLIBDIR "${CONCERTDIR}/lib/x86-64_sles10_4.1/static_pic")


#specific compiler settings
set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -pedantic -fopenmp -mfpmath=sse -O3 -m64 -pipe -fmessage-length=0 -D_FORTIFY_SOURCE=2 -DILOUSESTL -DILOSTRINGSTL -DIL_STD -D_GLIBCXX_DEBUG -fstack-protector -funwind-tables -fasynchronous-unwind-tables -DILOUSESTL -DILOSTRINGSTL -DIL_STD -D_REENTRANT -lz -lpng -DNO_FREETYPE -Wno-write-strings -std=c++0x  -I${CONCERTINCDIR} -I${CPLEXINCDIR} -I. -I./blossom5-v2.03/INCLUDE")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-g -lilocplex -lconcert -lcplex  -lpthread -lboost_regex  -DILOUSESTL -DILOSTRINGSTL -DIL_STD -lpthread  -L${CONCERTLIBDIR} -L${CPLEXLIBDIR} -L. -lrt")


 
# collect all sources (copy from 'ls -1 *.cpp *.h')
SET(2DVS_Sources
DisjointSet.h
DisjointSet.cpp
Main.cpp
Main.h
MIP.h
MIP.cpp
generalizedVRP_MIP.h
generalizedVRP_MIP.cpp
m_TSP_MIP.h
m_TSP_MIP.cpp
my_PM_test.cpp
Instance.h
Instance.cpp
Job.h
Job.cpp
Tours.h
)

# collect all sources
SET(GTestSources
datastructure_unittest.cpp
Job.h
Job.cpp
Instance.h
Instance.cpp
MIP.h
MIP.cpp
Tours.h
)


set(FindGraphConfiguation_sources
stringify.h
find_graph_configuration.cpp
)

#set_source_files_properties( MIP.cpp PROPERTIES CMAKE_CXX_FLAGS "-W0" ) 

# display status message for important variables
MESSAGE( STATUS )
MESSAGE( STATUS "-------------------------------------------------------------------------------" )
MESSAGE( STATUS "compiling with flags: " )
MESSAGE( STATUS "${CMAKE_CXX_FLAGS}" )
MESSAGE( STATUS "-------------------------------------------------------------------------------" )
MESSAGE( STATUS )

MESSAGE( STATUS )
MESSAGE( STATUS "-------------------------------------------------------------------------------" )
MESSAGE( STATUS "linking with flags: " )
MESSAGE( STATUS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS}" )
MESSAGE( STATUS "-------------------------------------------------------------------------------" )
MESSAGE( STATUS )

#define executable 
ADD_EXECUTABLE(2DVS ${2DVS_Sources} )
target_link_libraries (2DVS PM) 

ADD_EXECUTABLE(GTests ${GTestSources} )
target_link_libraries(GTests -lgtest -lgtest_main)

ADD_EXECUTABLE(find_graph_configuration ${FindGraphConfiguation_sources} )

