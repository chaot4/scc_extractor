# USING cmake:
#   mkdir build
#   cd build
#   cmake ..
#   make -j

# RUN unit tests:
#   cd build
#   ctest



# OBJECT library requires 2.8.8
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
project(scc_extractor CXX)

enable_testing()

include(CMakeDetermineCXXCompiler)

# default to RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
	set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	set(EXTRA_CXX_FLAGS "-Wall -Wextra -Wno-unused-parameter -pedantic" CACHE STRING "Extra flags used by the compiler during all build types.")
	set(EXTRA_EXE_LINKER_FLAGS "-Wl,--as-needed" CACHE STRING "Extra flags used by the linker.")
else("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	set(EXTRA_CXX_FLAGS "" CACHE STRING "Extra flags used by the compiler during all build types.")
	set(EXTRA_EXE_LINKER_FLAGS "" CACHE STRING "Extra flags used by the linker.")
endif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS} -std=c++11 -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_EXE_LINKER_FLAGS} -fopenmp")

option(VERBOSE "Verbose logging" OFF)

if(NOT VERBOSE)
	add_definitions(-DNVERBOSE)
endif(NOT VERBOSE)

# compile shared sources only once, and reuse object files in both,
# as they are compiled with the same options anyway
add_library(common OBJECT
	src/nodes_and_edges.cpp
	src/file_formats.cpp
)

add_executable(scc_extractor
	src/scc_extractor.cpp
	$<TARGET_OBJECTS:common>
)

add_executable(run_tests
	src/run_tests.cpp
	src/unit_tests.cpp
	$<TARGET_OBJECTS:common>
)

add_test(NAME unit-test
	WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/src"
	COMMAND $<TARGET_FILE:run_tests>
)
