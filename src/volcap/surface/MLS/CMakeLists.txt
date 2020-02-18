cmake_minimum_required (VERSION 3.8)

# set the project name
project(MLS)

# specify the c++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

# find and loads settings from PCL
# sets:
#   - PCL_FOUND, 
#   - PCL_INCLUDE_DIRS (dirs for PCL + 3rd party includes)
#   - PCL_LIBRARIES (file names of the built PCL libs)
#   - PCL_LIBRARY_DIRS (dirs for PCL + 3rd party dep libs)
#   - PCL_DEFINITIONS (needed preprocessor defs and compiler flags)
find_package(PCL 1.9 REQUIRED COMPONENTS common io surface)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add the executable
add_executable(MLS "MLS.cpp" "../SPSR/SPSR.cpp")

# specify include dirs for given target
target_link_libraries(MLS ${PCL_LIBRARIES})
target_include_directories(MLS PUBLIC "../SPSR")
