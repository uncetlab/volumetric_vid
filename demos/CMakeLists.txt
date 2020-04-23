cmake_minimum_required (VERSION 3.8)

project(demos)

# specify the c++ standard
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PCL
# sets:
#   - PCL_FOUND, 
#   - PCL_INCLUDE_DIRS (dirs for PCL + 3rd party includes)
#   - PCL_LIBRARIES (file names of the built PCL libs)
#   - PCL_LIBRARY_DIRS (dirs for PCL + 3rd party dep libs)
#   - PCL_DEFINITIONS (needed preprocessor defs and compiler flags)
find_package(PCL 1.9.1 REQUIRED COMPONENTS common io surface visualization features)  #COMPONENTS common io surface visualization features
message(STATUS "PCL_INCLUDE_DIRS=${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS=${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_DEFINITIONS=${PCL_DEFINITIONS}")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#link_libraries(PRIVATE volcap_library ${PCL_LIBRARIES})

add_executable(DemoMLS "demo_mls.cpp")
add_executable(DemoSPSR "demo_spsr.cpp")
add_executable(DemoUVMap "demo_uv_map.cpp")
add_executable(DemoTexturing "demo_texturing.cpp")
add_executable(DemoFbx "demo_fbx.cpp")
add_executable(DemoUSD "demo_usd.cpp")
add_executable(DemoVisualizer "demo_visualizer.cpp")
add_executable(DemoVCLData "demo_vcl_data.cpp")

# these are just me messing around with libs, shouldn't be included  # TODO: find a place for them
#add_executable(DemoBlend2D "demo_blend2d.cpp")
#add_executable(DemoUSDCrate "demo_usd_crate.cpp")
#add_executable(DemoUSDBillboard "demo_usd_billboard.cpp")

# include volcap for all targets
target_link_libraries(DemoMLS PRIVATE volcap_library ${PCL_LIBRARIES})
target_link_libraries(DemoSPSR PRIVATE volcap_library ${PCL_LIBRARIES})
target_link_libraries(DemoUVMap PRIVATE volcap_library ${PCL_LIBRARIES})
target_link_libraries(DemoTexturing PRIVATE volcap_library ${PCL_LIBRARIES})
target_link_libraries(DemoFbx PRIVATE volcap_library ${PCL_LIBRARIES})
target_link_libraries(DemoUSD PRIVATE volcap_library ${PCL_LIBRARIES})
target_link_libraries(DemoVisualizer PRIVATE volcap_library)
target_link_libraries(DemoVCLData PRIVATE ${PCL_LIBRARIES})
