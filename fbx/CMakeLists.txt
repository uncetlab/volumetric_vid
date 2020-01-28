cmake_minimum_required (VERSION 3.8)

# set the project name
project(FBX)

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
#find_package(PCL 1.9 REQUIRED COMPONENTS common io surface visualization features)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#set(FBXSDK_LIBRARY "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/release/libfbxsdk-md.lib")
set(FBXSDK_LIBRARY "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/debug/libfbxsdk-md.lib")
set(FBXSDK_INCLUDE_PATH "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/include")
message("FBXSDK_LIBRARY = '${FBXSDK_LIBRARY}'")
message("FBXSDK_INCLUDE_PATH = '${FBXSDK_INCLUDE_PATH}'")

# include headers - for all executables?
include_directories(${FBXSDK_INCLUDE_PATH})

# add the executable
add_executable(FBXFirstProgram "fbx_first_program.cpp")
add_executable(CreateCrateFBX "create_crate_fbx.cpp")

# specify include dirs for given target

#target_link_libraries(FBXFirstProgram ${PCL_LIBRARIES})
target_link_libraries(FBXFirstProgram ${FBXSDK_LIBRARY})
#target_link_libraries(FBXFirstProgram "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/release/libxml2-md.lib")
#target_link_libraries(FBXFirstProgram "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/release/zlib-md.lib")
target_link_libraries(FBXFirstProgram "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/debug/libxml2-md.lib")
target_link_libraries(FBXFirstProgram "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/debug/zlib-md.lib")


target_link_libraries(CreateCrateFBX ${FBXSDK_LIBRARY})
target_link_libraries(CreateCrateFBX "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/debug/libxml2-md.lib")
target_link_libraries(CreateCrateFBX "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/debug/zlib-md.lib")

