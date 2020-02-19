cmake_minimum_required (VERSION 3.8)

project(demos)

# specify the c++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# load PCL for all targets
# find and loads settings from PCL
# sets:
#   - PCL_FOUND, 
#   - PCL_INCLUDE_DIRS (dirs for PCL + 3rd party includes)
#   - PCL_LIBRARIES (file names of the built PCL libs)
#   - PCL_LIBRARY_DIRS (dirs for PCL + 3rd party dep libs)
#   - PCL_DEFINITIONS (needed preprocessor defs and compiler flags)
find_package(PCL 1.3 REQUIRED COMPONENTS common io surface visualization features)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(BLEND2D_LIBRARY_DEBUG "C:/Users/maxhu/repos/blend2d/blend2d/build64/Debug/blend2d.lib")		# must also copy .dll into CMakeBuilds!!
set(BLEND2D_LIBRARY_RELEASE "C:/Users/maxhu/repos/blend2d/blend2d/build64/Release/blend2d.lib") # must also copy .dll into CMakeBuilds!!
set(BLEND2D_INCLUDE_PATH "C:/Users/maxhu/repos/blend2d/blend2d/src")

# include volcap for all targets
include_directories("../src")

# add targets
add_executable(DemoSPSR "demo_spsr.cpp" "../src/volcap/surface/spsr.cpp")
add_executable(DemoMLS "demo_mls.cpp" "../src/volcap/surface/spsr.cpp")
add_executable(DemoVisualizer "demo_visualizer.cpp" "../src/volcap/viz/viz.cpp" "../src/volcap/io/io.cpp")
add_executable(DemoTexturing "demo_texturing.cpp" "../src/volcap/texture/texturing.cpp" "../src/volcap/io/io.cpp" "../src/volcap/io/io_cam.cpp")
add_executable(DemoBlend2D "demo_blend2d.cpp")
#add_executable(pcl_visualizer_demo "pcl_visualizer_demo.cpp")

# specify include dirs for given target
target_include_directories(DemoTexturing PRIVATE ${BLEND2D_INCLUDE_PATH})
target_include_directories(DemoBlend2D PRIVATE ${BLEND2D_INCLUDE_PATH})

# specify libraries for given target
target_link_libraries(DemoSPSR ${PCL_LIBRARIES})
target_link_libraries(DemoMLS ${PCL_LIBRARIES})
target_link_libraries(DemoVisualizer ${PCL_LIBRARIES})
target_link_libraries(DemoTexturing ${PCL_LIBRARIES})
#target_link_libraries(DemoTexturing ${BLEND2D_LIBRARY_DEBUG})
target_link_libraries(DemoBlend2D ${BLEND2D_LIBRARY_RELEASE})
#target_link_libraries(DemoBlend2D ${BLEND2D_LIBRARY_DEBUG})
target_link_libraries(DemoBlend2D ${BLEND2D_LIBRARY_RELEASE})
#target_link_libraries(pcl_visualizer_demo ${PCL_LIBRARIES})
