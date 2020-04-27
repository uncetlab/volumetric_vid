

#including pcl necessary? need boost program options
# PCL
# sets:
#   - PCL_FOUND, 
#   - PCL_INCLUDE_DIRS (dirs for PCL + 3rd party includes)
#   - PCL_LIBRARIES (file names of the built PCL libs)
#   - PCL_LIBRARY_DIRS (dirs for PCL + 3rd party dep libs)
#   - PCL_DEFINITIONS (needed preprocessor defs and compiler flags)
#find_package(PCL 1.9.1 REQUIRED COMPONENTS common io surface visualization features)  #COMPONENTS common io surface visualization features
#message(STATUS "PCL_INCLUDE_DIRS=${PCL_INCLUDE_DIRS}")
#message(STATUS "PCL_LIBRARY_DIRS=${PCL_LIBRARY_DIRS}")
#message(STATUS "PCL_DEFINITIONS=${PCL_DEFINITIONS}")
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

set(BOOST_ROOT "C:/Program Files/Boost")
#set(BOOST_LIBRARYDIR "C:/Program Files/Boost/lib")
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
#set(BOOST_ROOT "C:/Program Files/boost_1_72_0")
find_package(Boost REQUIRED COMPONENTS program_options)
message(STATUS "Boost_FOUND=${Boost_FOUND}")
message(STATUS "Boost_INCLUDE_DIRS=${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARY_DIRS=${Boost_LIBRARY_DIRS}")
message(STATUS "Boost_LIBRARIES=${Boost_LIBRARIES}")
#include_directories(${Boost_INCLUDE_DIRS})

add_executable(volcap_app "volcap.cpp")

target_link_libraries(volcap_app PRIVATE volcap_library Boost::program_options)
#target_link_libraries(volcap_app PRIVATE volcap_library ${PCL_LIBRARIES})  #  ${PCL_LIBRARIES}
