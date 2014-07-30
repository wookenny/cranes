cmake_minimum_required(VERSION 2.6)

#project name
project(2D-VehicleScheduling)

ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/blossom5-v2.03/SRC)

#some path settings
set(CPLEXBASE "/usr/site-local/ILOG/CPLEX_Studio126")
set(CPLEXDIR "${CPLEXBASE}/cplex")
set(CPLEXINCDIR "${CPLEXDIR}/include")
set(CPLEXLIBDIR "${CPLEXDIR}/lib/x86-64_linux/static_pic")

set(CONCERTDIR "${CPLEXBASE}/concert")
set(CONCERTINCDIR "${CONCERTDIR}/include")
set(CONCERTLIBDIR "${CONCERTDIR}/lib/x86-64_linux/static_pic")

FIND_PACKAGE( Boost  COMPONENTS program_options filesystem iostreams system regex REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )


set(twoDVS_sourcesDir "${CMAKE_CURRENT_SOURCE_DIR}/src")  
set(MATCHING_INCDIR "${CMAKE_CURRENT_SOURCE_DIR}/blossom5-v2.03/INCLUDE")  
#specific compiler settings

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wpointer-arith -Wformat-nonliteral -Winit-self -Wall -Wextra -pedantic -m64 -pipe -fmessage-length=0  -fstack-protector -funwind-tables -fasynchronous-unwind-tables -Wno-write-strings -std=c++11")
#deleted: -mfpmath=sse+387

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -Wpointer-arith -Wformat-nonliteral -Winit-self -Wall -Wextra -pedantic -m64 -pipe -fmessage-length=0  -funwind-tables -fasynchronous-unwind-tables -Wno-write-strings -std=c++11 ")

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -D _GLIBCXX_DEBUG -D #_GLIBCXX_DEBUG_PEDANTIC")

set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS " -Wl,--discard-all -Wl,--no-undefined -Wl,--build-id=sha1 -Wl,-O3 -rdynamic ")

INCLUDE_DIRECTORIES(/usr/site-local/include ${CPLEXINCDIR} ${CONCERTINCDIR} ${MATCHING_INCDIR} .)
LINK_DIRECTORIES(/usr/site-local/lib ${CONCERTLIBDIR} ${CPLEXLIBDIR})


add_definitions(-DILOSTRINGSTL -DIL_STD -D_FORTIFY_SOURCE=2  -DILOUSESTL  -D_REENTRANT -DNO_FREETYPE )

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")

# collect all sources (copy from 'ls -1 *.cpp *.h')
file(GLOB 2DVS_Sources
    ${twoDVS_sourcesDir}/*.h
    ${twoDVS_sourcesDir}/*.cpp
)

# collect all sources
file(GLOB 2DVS_Sources
    ${twoDVS_sourcesDir}/*.h
    ${twoDVS_sourcesDir}/*.cpp
)

# collect all sources
file(GLOB GTests_Sources
    ${twoDVS_sourcesDir}/*.h
    ${twoDVS_sourcesDir}/*.cpp
)

set(FindGraphConfiguation_sources
${twoDVS_sourcesDir}/stringify.h
${twoDVS_sourcesDir}/find_graph_configuration.cpp
)

set(Versioning_sources
build_version.sh
)

list(REMOVE_ITEM 2DVS_Sources "${twoDVS_sourcesDir}/find_graph_configuration.cpp" "${twoDVS_sourcesDir}/datastructure_unittest.cpp" "${twoDVS_sourcesDir}/Common_Unittest.cpp" )
ADD_EXECUTABLE(2DVS ${2DVS_Sources}  )
target_link_libraries (2DVS PM pthread ilocplex concert cplex ${Boost_LIBRARIES} )

add_custom_target(
    myBuildVersion
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/build/build_version.sh ${CMAKE_CURRENT_SOURCE_DIR}/src/version.txt ${CMAKE_BUILD_TYPE}
    )
add_dependencies(2DVS myBuildVersion)



list(REMOVE_ITEM GTests_Sources "${twoDVS_sourcesDir}/find_graph_configuration.cpp" "${twoDVS_sourcesDir}/Main.cpp")
ADD_EXECUTABLE(GTests ${GTests_Sources} )
TARGET_LINK_LIBRARIES(GTests PM pthread gtest  gtest_main pthread ilocplex concert cplex ${Boost_LIBRARIES})
set_target_properties (GTests PROPERTIES COMPILE_DEFINITIONS "GTESTS_ENABLED")
add_custom_command(TARGET GTests
                   POST_BUILD
                   COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/bin/GTests)


ADD_EXECUTABLE(find_graph_configuration ${FindGraphConfiguation_sources} )
TARGET_LINK_LIBRARIES(find_graph_configuration ilocplex concert cplex)
