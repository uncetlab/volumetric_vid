cmake_minimum_required (VERSION 3.8)

# set the project name
project(Texturing)

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
find_package(PCL 1.9 REQUIRED COMPONENTS common io surface visualization features)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_path(BLEND2D_INCLUDE_DIR blend2d.h PATHS C:/Users/maxhu/repos/blend2d/blend2d/src)
#find_library(BLEND2D_LIBRARY blend2d)
#if(NOT BLEND2D_INCLUDE_DIR)
#  message(FATAL_ERROR "BLEND2D_INCLUDE_DIR not found")
#endif()
#if(NOT BLEND2D_LIBRARY)
#  message(FATAL_ERROR "BLEND2D_LIBRARY not found")
#endif()

#if (BLEND2D_INCLUDE_DIR AND BLEND2D_LIBRARY)
    # you may need that if further action in your CMakeLists.txt depends
    # on detecting your library
#    set(BLEND2D_FOUND TRUE)

    # you may need that if you want to conditionally compile some parts
    # of your code depending on library availability
#    add_definitions(-DHAVE_BLEND2D=1)

    # those two, you really need
#    include_directories(${BLEND2D_INCLUDE_DIR})
#    set(YOUR_LIBRARIES ${YOUR_LIBRARIES} ${BLEND2D_LIBRARY})
#endif ()

set(BLEND2D_LIBRARY "C:/Users/maxhu/repos/blend2d/blend2d/build/Debug/blend2d.lib")
set(BLEND2D_INCLUDE_PATH "C:/Users/maxhu/repos/blend2d/blend2d/src")
#message(FATAL_ERROR "BLEND2D_INCLUDE_DIR not found:")
message("BLEND2D_LIBRARY = '${BLEND2D_LIBRARY}'")
message("BLEND2D_INCLUDE_PATH = '${BLEND2D_INCLUDE_PATH}'")
message("HI")

include_directories(${BLEND2D_INCLUDE_PATH})


# add the executable
add_executable(Texturing "texture_mapping.cpp")
add_executable(CustomTexturing "custom_texture_mapping.cpp")
add_executable(Blend2D_test "blend2d_test.cpp")

# specify include dirs for given target
target_link_libraries(Texturing ${PCL_LIBRARIES})
target_link_libraries(CustomTexturing ${PCL_LIBRARIES})
target_link_libraries(CustomTexturing ${BLEND2D_LIBRARY})
target_link_libraries(Blend2D_test ${BLEND2D_LIBRARY})
