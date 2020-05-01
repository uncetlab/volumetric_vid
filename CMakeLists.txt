cmake_minimum_required (VERSION 3.8)

project ("VolumetricVidCMake")

# define a preprocessor variable to get the location of the project's root directory
add_definitions( -D_PROJECT_DIR=\"${CMAKE_SOURCE_DIR}\" )

# The compiled library code is here
add_subdirectory(src)

# The demo code is here
add_subdirectory(demos)

# The command line tools are here
add_subdirectory ("apps")
