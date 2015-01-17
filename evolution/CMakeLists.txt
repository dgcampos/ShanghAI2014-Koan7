# Alternative compilations:
#########################################################################################
# For release build, run cmake with this arguments:
#-DCMAKE_BUILD_TYPE=RELEASE
# For debugging purposes compilation, run cmake with this arguments:
#-DCMAKE_BUILD_TYPE=DEBUG
# For debugging and profiling purposes, run cmake with this arguments:
#-DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_CXX_FLAGS=-pg


cmake_minimum_required (VERSION 2.6.0)

# Configure the project:
PROJECT( EvolveGaits )

set(CMAKE_BUILD_TYPE RelWithDebInfo)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# Dependencies
###################################################

# ECF -> Evolutionary Computation Framework
set( ECF_LINK_FLAGS "-lecf" )



# Set output dirs
set (EvolveGaits_TRUNK_DIR "${PROJECT_SOURCE_DIR}")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")


# Create output dirs
make_directory(${LIBRARY_OUTPUT_PATH})
make_directory(${EXECUTABLE_OUTPUT_PATH})

# Code is in src folder
add_subdirectory(src)






