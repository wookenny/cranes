cmake_minimum_required(VERSION 2.6)

#project name
project(2D-VehicleScheduling)

ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/blossom5-v2.03/SRC)

#some path settings
set(CPLEXDIR "/usr/site-local/ILOG/CPLEX_Studio124/cplex")
set(CPLEXINCDIR "${CPLEXDIR}/include")
set(CPLEXLIBDIR "${CPLEXDIR}/lib/x86-64_sles10_4.1/static_pic")

set(CONCERTDIR "/usr/site-local/ILOG/CPLEX_Studio124/concert")
set(CONCERTINCDIR "${CONCERTDIR}/include")
set(CONCERTLIBDIR "${CONCERTDIR}/lib/x86-64_sles10_4.1/static_pic")

set(twoDVS_sourcesDir "${CMAKE_CURRENT_SOURCE_DIR}/src")  
set(MATCHING_INCDIR "${CMAKE_CURRENT_SOURCE_DIR}/blossom5-v2.03/INCLUDE")  
#specific compiler settings

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -fopenmp -mfpmath=sse+387 -m64 -pipe -fmessage-length=0 -D_FORTIFY_SOURCE=2 -DILOUSESTL -DILOSTRINGSTL -DIL_STD -fstack-protector -funwind-tables -fasynchronous-unwind-tables -DILOUSESTL -DILOSTRINGSTL -DIL_STD -D_REENTRANT -DNO_FREETYPE -Wno-write-strings -std=c++0x  -I${CONCERTINCDIR} -I/usr/site-local/include -I${CPLEXINCDIR} -I. -I${MATCHING_INCDIR} ")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS " -lilocplex -lconcert -lcplex  -lpthread -lboost_regex  -DILOUSESTL -DILOSTRINGSTL -DIL_STD -lpthread  -L${CONCERTLIBDIR} -L${CPLEXLIBDIR} -L. -L/usr/site-local/lib -lrt -lz  -lboost_iostreams -lboost_program_options -lboost_filesystem -lboost_system")


#clang is more silent if this is set: -Qunused-arguments
 
# collect all sources (copy from 'ls -1 *.cpp *.h')
SET(2DVS_Sources
${twoDVS_sourcesDir}/Common.h
${twoDVS_sourcesDir}/Common.cpp
${twoDVS_sourcesDir}/DisjointSet.h
${twoDVS_sourcesDir}/DisjointSet.cpp
${twoDVS_sourcesDir}/Main.cpp
${twoDVS_sourcesDir}/Main.h
${twoDVS_sourcesDir}/SingleCraneTSP_Solver.h
${twoDVS_sourcesDir}/SingleCraneTSP_Solver.cpp
${twoDVS_sourcesDir}/generalizedVRP_MIP.h
${twoDVS_sourcesDir}/generalizedVRP_MIP.cpp
${twoDVS_sourcesDir}/m_TSP_MIP.h
${twoDVS_sourcesDir}/m_TSP_MIP.cpp
${twoDVS_sourcesDir}/independent_TSP_MIP.h
${twoDVS_sourcesDir}/independent_TSP_MIP.cpp
${twoDVS_sourcesDir}/InsertionHeuristic.h
${twoDVS_sourcesDir}/InsertionHeuristic.cpp
${twoDVS_sourcesDir}/my_PM_test.cpp
${twoDVS_sourcesDir}/Instance.h
${twoDVS_sourcesDir}/Instance.cpp
${twoDVS_sourcesDir}/Point.h
${twoDVS_sourcesDir}/Point.cpp
${twoDVS_sourcesDir}/Job.h
${twoDVS_sourcesDir}/Job.cpp
${twoDVS_sourcesDir}/Tours.h
${twoDVS_sourcesDir}/SingleCraneTourApproximation.cpp
${twoDVS_sourcesDir}/LaserSharingProblemWriter.h
${twoDVS_sourcesDir}/LaserSharingProblemWriter.cpp
)

# collect all sources
SET(GTestSources
${twoDVS_sourcesDir}/Common.h
${twoDVS_sourcesDir}/Common.cpp
${twoDVS_sourcesDir}/Common_Unittest.cpp
${twoDVS_sourcesDir}/datastructure_unittest.cpp
${twoDVS_sourcesDir}/DisjointSet.h
${twoDVS_sourcesDir}/DisjointSet.cpp
${twoDVS_sourcesDir}/Job.h
${twoDVS_sourcesDir}/Job.cpp
${twoDVS_sourcesDir}/Instance.h
${twoDVS_sourcesDir}/Instance.cpp
${twoDVS_sourcesDir}/InsertionHeuristic.h
${twoDVS_sourcesDir}/InsertionHeuristic.cpp
${twoDVS_sourcesDir}/Point.h
${twoDVS_sourcesDir}/Point.cpp
${twoDVS_sourcesDir}/generalizedVRP_MIP.h
${twoDVS_sourcesDir}/generalizedVRP_MIP.cpp
${twoDVS_sourcesDir}/m_TSP_MIP.h
${twoDVS_sourcesDir}/m_TSP_MIP.cpp
${twoDVS_sourcesDir}/independent_TSP_MIP.h
${twoDVS_sourcesDir}/independent_TSP_MIP.cpp
${twoDVS_sourcesDir}/Tours.h
)


set(FindGraphConfiguation_sources
${twoDVS_sourcesDir}/stringify.h
${twoDVS_sourcesDir}/find_graph_configuration.cpp
)

set(Versioning_sources
build_version.sh
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

ADD_EXECUTABLE(2DVS ${2DVS_Sources} )
target_link_libraries (2DVS PM) 

add_custom_target(
    myBuildVersion
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/build/build_version.sh ${CMAKE_CURRENT_SOURCE_DIR}/src/version.txt ${CMAKE_BUILD_TYPE}
    )
add_dependencies(2DVS myBuildVersion)


ADD_EXECUTABLE(GTests ${GTestSources} )
target_link_libraries(GTests -lgtest -lgtest_main)
set_target_properties (GTests PROPERTIES COMPILE_DEFINITIONS "GTESTS_ENABLED")

ADD_EXECUTABLE(find_graph_configuration ${FindGraphConfiguation_sources} )

