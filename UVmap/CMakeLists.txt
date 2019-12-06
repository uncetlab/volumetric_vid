cmake_minimum_required (VERSION 3.8)

# set the project name
project(UVmap)


#
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
#find_package(PCL 1.3 REQUIRED COMPONENTS common io surface)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# add the executable
add_executable(UVmap "UVmap.cpp")

# specify include dirs for given target
target_include_directories(UVmap PUBLIC C:\\Users\\maxhu\\repos\\Microsoft_UVAtlas\\UVAtlas\\UVAtlas\\inc)
target_include_directories(UVmap PUBLIC C:\\Users\\maxhu\\repos\\Microsoft_UVAtlas\\DirectXMesh\\Utilities)
target_include_directories(UVmap PUBLIC C:\\Users\\maxhu\\repos\\Microsoft_UVAtlas\\DirectXMesh\\DirectXMesh)

target_link_libraries(UVmap C:\\Users\\maxhu\\repos\\Microsoft_UVAtlas\\UVAtlas\\UVAtlas\\Bin\\Desktop_2017_Win10\\x64\\Debug\\UVAtlas.lib)
target_link_libraries(UVmap C:\\Users\\maxhu\\repos\\Microsoft_UVAtlas\\DirectXMesh\\DirectXMesh\\Bin\\Desktop_2017\\x64\\Debug\\DirectXMesh.lib)

