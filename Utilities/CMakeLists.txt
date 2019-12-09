cmake_minimum_required (VERSION 3.8)

# set the project name
project(Utilities)

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
find_package(PCL 1.9 REQUIRED COMPONENTS common io surface visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#set(BOOST_ROOT "C:\\Program Files\\PCL 1.9.1\\3rdParty\\Boost")
#set(BOOST_INCLUDEDIR "C:\\Program Files\\PCL 1.9.1\\3rdParty\\Boost\\include")
#set(BOOST_LIBRARYDIR "C:\\Program Files\\PCL 1.9.1\\3rdParty\\Boost\\lib")
#find_package(Boost 1.68 REQUIRED)
#include_directories(${Boost_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARY_DIRS})
#add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})

# add the executable
#add_executable(WriteUVJPEG "write_uv_jpeg.cpp")
add_executable(exTexMesh "example_texture_mesh.cpp")

# specify include dirs for given target
target_link_libraries(exTexMesh ${PCL_LIBRARIES})
#target_link_libraries(WriteUVJPEG ${Boost_LIBRARIES})

