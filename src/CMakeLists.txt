# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.

###### Optionally glob, but only for CMake 3.12 or later:
# file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${ModernCMakeExample_SOURCE_DIR}/include/modern/*.hpp")
#set(HEADER_LIST "${ModernCMakeExample_SOURCE_DIR}/include/modern/lib.hpp")

file(GLOB_RECURSE VOLCAP_SRC CONFIGURE_DEPENDS
    "*.h"
    "*.cpp"
)
message(STATUS "VOLCAP_SRC=${VOLCAP_SRC}")

###### Make an automatic library - will be static or dynamic based on user setting
#add_library(modern_library lib.cpp ${HEADER_LIST})
#add_library(volcap_library ${VOLCAP_SRC})
add_library(volcap_library STATIC ${VOLCAP_SRC})

###### We need this directory, and users of our library will need it too
#target_include_directories(modern_library PUBLIC ../include)
target_include_directories(volcap_library PUBLIC ./)

###### This depends on...

################################################################################################
# Finding dependencies
################################################################################################

# PCL
# sets:
#   - PCL_FOUND, 
#   - PCL_INCLUDE_DIRS (dirs for PCL + 3rd party includes)
#   - PCL_LIBRARIES (file names of the built PCL libs)
#   - PCL_LIBRARY_DIRS (dirs for PCL + 3rd party dep libs)
#   - PCL_DEFINITIONS (needed preprocessor defs and compiler flags)
find_package(PCL 1.9.1 REQUIRED COMPONENTS common io surface visualization features)
message(STATUS "PCL_INCLUDE_DIRS=${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS=${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_DEFINITIONS=${PCL_DEFINITIONS}")
include_directories(${PCL_INCLUDE_DIRS}) # TODO: are these 3 necessary?
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# UVAtlas
find_path(UVATLAS_INCLUDE_DIR UVAtlas.h HINTS "${CMAKE_SOURCE_DIR}/external/UVAtlas/UVAtlas/inc" NO_DEFAULT_PATH)
find_library(UVATLAS_LIBRARY_DEBUG UVAtlas HINTS "${CMAKE_SOURCE_DIR}/external/UVAtlas/UVAtlas/Bin/Desktop_2017_Win10/x64/Debug" NO_DEFAULT_PATH)
find_library(UVATLAS_LIBRARY_RELEASE UVAtlas HINTS "${CMAKE_SOURCE_DIR}/external/UVAtlas/UVAtlas/Bin/Desktop_2017_Win10/x64/Release" NO_DEFAULT_PATH)
message(STATUS "UVATLAS_INCLUDE_DIR=${UVATLAS_INCLUDE_DIR}")
message(STATUS "UVATLAS_LIBRARY_DEBUG=${UVATLAS_LIBRARY_DEBUG}")
message(STATUS "UVATLAS_LIBRARY_RELEASE=${UVATLAS_LIBRARY_RELEASE}")

# DirectXMesh
find_path(DIRECTXMESH_INCLUDE_DIR DirectXMesh.h HINTS "${CMAKE_SOURCE_DIR}/external/DirectXMesh/DirectXMesh" NO_DEFAULT_PATH)
find_path(WAVEFRONTREADER_INCLUDE_DIR WaveFrontReader.h HINTS "${CMAKE_SOURCE_DIR}/external/DirectXMesh/Utilities" NO_DEFAULT_PATH)
find_library(DIRECTXMESH_LIBRARY_DEBUG DirectXMesh HINTS "${CMAKE_SOURCE_DIR}/external/DirectXMesh/DirectXMesh/Bin/Desktop_2017_Win10/x64/Debug" NO_DEFAULT_PATH)
find_library(DIRECTXMESH_LIBRARY_RELEASE DirectXMesh HINTS "${CMAKE_SOURCE_DIR}/external/DirectXMesh/DirectXMesh/Bin/Desktop_2017_Win10/x64/Release" NO_DEFAULT_PATH)
message(STATUS "DIRECTXMESH_INCLUDE_DIR=${DIRECTXMESH_INCLUDE_DIR}")
message(STATUS "DIRECTXMESH_LIBRARY_DEBUG=${DIRECTXMESH_LIBRARY_DEBUG}")
message(STATUS "DIRECTXMESH_LIBRARY_RELEASE=${DIRECTXMESH_LIBRARY_RELEASE}")

# Blend2D
set(BLEND2D_STATIC TRUE)
set(BLEND2D_DIR ${CMAKE_SOURCE_DIR}/external/blend2d)
include("${BLEND2D_DIR}/CMakeLists.txt")
message(STATUS "BLEND2D_INCLUDE_DIRS=${BLEND2D_INCLUDE_DIRS}")

# FBX SDK
find_path(FBXSDK_INCLUDE_PATH fbxsdk.h "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/include" NO_DEFAULT_PATH)
set(FBXSDK_LIBRARIES_RELEASE "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/release/libfbxsdk-md.lib;C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/release/libxml2-md.lib;C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/release/zlib-md.lib")
set(FBXSDK_LIBRARIES_DEBUG "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/debug/libfbxsdk-md.lib;C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/debug/libxml2-md.lib;C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/debug/zlib-md.lib")

# USD SDK
set(pxr_DIR "C:/Program Files/USD")
find_package(pxr REQUIRED CONFIG)
message(STATUS "PXR_arch_LIBRARY=${PXR_arch_LIBRARY}")
message(STATUS "PXR_LIBRARIES=${PXR_LIBRARIES}")
message(STATUS "PXR_INCLUDE_DIRS=${PXR_INCLUDE_DIRS}")

target_include_directories(
	volcap_library
	PUBLIC ${FBXSDK_INCLUDE_PATH}
	PUBLIC ${PXR_INCLUDE_DIRS}
	PRIVATE ${UVATLAS_INCLUDE_DIR}
	PRIVATE ${DIRECTXMESH_INCLUDE_DIR}
	PRIVATE ${WAVEFRONTREADER_INCLUDE_DIR}
	PRIVATE ${BLEND2D_INCLUDE_DIR}
)


target_link_libraries(
	volcap_library
	general ${PCL_LIBRARIES}
	general ${PXR_LIBRARIES}
	general Blend2D::Blend2D
	debug ${UVATLAS_LIBRARY_DEBUG}
	debug ${DIRECTXMESH_LIBRARY_DEBUG}
	debug ${FBXSDK_LIBRARIES_DEBUG}
	optimized ${UVATLAS_LIBRARY_RELEASE}
	optimized ${DIRECTXMESH_LIBRARY_RELEASE}
	optimized ${FBXSDK_LIBRARIES_RELEASE}
)

################################################################################################
# dependencies done
################################################################################################

###### All users of this library will need at least C++11
target_compile_features(volcap_library PUBLIC cxx_std_11)

###### IDEs should put the headers in a nice place
#source_group(TREE "${PROJECT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${HEADER_LIST})
