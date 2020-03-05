cmake_minimum_required (VERSION 3.8)

project ("VolumetricVidCMake")

add_definitions( -D_PROJECT_DIR=\"${CMAKE_SOURCE_DIR}\" )

add_subdirectory ("demos")
