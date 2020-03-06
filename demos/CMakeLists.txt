cmake_minimum_required (VERSION 3.8)

project(demos)

# specify the c++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
find_package(PCL 1.9.1 REQUIRED COMPONENTS common io surface visualization features)  #COMPONENTS common io surface visualization features
message("PCL_INCLUDE_DIRS=${PCL_INCLUDE_DIRS}")
message("PCL_LIBRARY_DIRS=${PCL_LIBRARY_DIRS}")
message("PCL_DEFINITIONS=${PCL_DEFINITIONS}")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# UVAtlas
find_path(UVATLAS_INCLUDE_DIR UVAtlas.h HINTS "${CMAKE_SOURCE_DIR}/external/UVAtlas/UVAtlas/inc" NO_DEFAULT_PATH)
find_library(UVATLAS_LIBRARY_DEBUG UVAtlas HINTS "${CMAKE_SOURCE_DIR}/external/UVAtlas/UVAtlas/Bin/Desktop_2017_Win10/x64/Debug" NO_DEFAULT_PATH)
find_library(UVATLAS_LIBRARY_RELEASE UVAtlas HINTS "${CMAKE_SOURCE_DIR}/external/UVAtlas/UVAtlas/Bin/Desktop_2017_Win10/x64/Release" NO_DEFAULT_PATH)
message("UVATLAS_INCLUDE_DIR=${UVATLAS_INCLUDE_DIR}")
message("UVATLAS_LIBRARY_DEBUG=${UVATLAS_LIBRARY_DEBUG}")
message("UVATLAS_LIBRARY_RELEASE=${UVATLAS_LIBRARY_RELEASE}")

# DirectXMesh
find_path(DIRECTXMESH_INCLUDE_DIR DirectXMesh.h HINTS "${CMAKE_SOURCE_DIR}/external/DirectXMesh/DirectXMesh" NO_DEFAULT_PATH)
find_path(WAVEFRONTREADER_INCLUDE_DIR WaveFrontReader.h HINTS "${CMAKE_SOURCE_DIR}/external/DirectXMesh/Utilities" NO_DEFAULT_PATH)
find_library(DIRECTXMESH_LIBRARY_DEBUG DirectXMesh HINTS "${CMAKE_SOURCE_DIR}/external/DirectXMesh/DirectXMesh/Bin/Desktop_2017_Win10/x64/Debug" NO_DEFAULT_PATH)
find_library(DIRECTXMESH_LIBRARY_RELEASE DirectXMesh HINTS "${CMAKE_SOURCE_DIR}/external/DirectXMesh/DirectXMesh/Bin/Desktop_2017_Win10/x64/Release" NO_DEFAULT_PATH)
message("DIRECTXMESH_INCLUDE_DIR=${DIRECTXMESH_INCLUDE_DIR}")
message("DIRECTXMESH_LIBRARY_DEBUG=${DIRECTXMESH_LIBRARY_DEBUG}")
message("DIRECTXMESH_LIBRARY_RELEASE=${DIRECTXMESH_LIBRARY_RELEASE}")

# Blend2D
find_path(BLEND2D_INCLUDE_DIR blend2d.h HINTS "${CMAKE_SOURCE_DIR}/external/blend2d/src" NO_DEFAULT_PATH)
find_library(BLEND2D_LIBRARY_DEBUG blend2d HINTS "${CMAKE_SOURCE_DIR}/external/blend2d/build/Debug" NO_DEFAULT_PATH)
find_library(BLEND2D_LIBRARY_RELEASE blend2d HINTS "${CMAKE_SOURCE_DIR}/external/blend2d/build/Release" NO_DEFAULT_PATH)
message("BLEND2D_INCLUDE_DIR=${BLEND2D_INCLUDE_DIR}")
message("BLEND2D_LIBRARY_DEBUG=${BLEND2D_LIBRARY_DEBUG}")
message("BLEND2D_LIBRARY_RELEASE=${BLEND2D_LIBRARY_RELEASE}")

# FBX SDK
find_path(FBXSDK_INCLUDE_PATH fbxsdk.h "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/include" NO_DEFAULT_PATH)
set(FBXSDK_LIBRARIES_RELEASE "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/release/libfbxsdk-md.lib;C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/release/libxml2-md.lib;C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/release/zlib-md.lib")
set(FBXSDK_LIBRARIES_DEBUG "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/debug/libfbxsdk-md.lib;C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/debug/libxml2-md.lib;C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/lib/vs2017/x64/debug/zlib-md.lib")


################################################################################################
# Adding targets
################################################################################################

#include volcap for all targets
include_directories("../src")

add_executable(DemoMLS "demo_mls.cpp" "../src/volcap/surface/spsr.cpp")
add_executable(DemoSPSR "demo_spsr.cpp" "../src/volcap/surface/spsr.cpp")
add_executable(DemoVisualizer "demo_visualizer.cpp" "../src/volcap/viz/viz.cpp" "../src/volcap/io/io.cpp")
add_executable(DemoTexturing "demo_texturing.cpp" "../src/volcap/texture/texturing.cpp" "../src/volcap/io/io.cpp" "../src/volcap/io/io_cam.cpp")
add_executable(DemoBlend2D "demo_blend2d.cpp")
add_executable(DemoUVMap "demo_uv_map.cpp" "../src/volcap/io/io.cpp" "../src/volcap/texture/uv_map.cpp")
add_executable(DemoFbx "demo_fbx.cpp" "../src/volcap/io/io.cpp" "../src/volcap/io/io_fbx.cpp")
#add_executable(pcl_visualizer_demo "pcl_visualizer_demo.cpp")

#### DemoMLS
target_link_libraries(DemoMLS ${PCL_LIBRARIES})

#### DemoSPSR
target_link_libraries(DemoSPSR ${PCL_LIBRARIES})

#### DemoUVMap
target_include_directories(DemoUVMap PRIVATE ${UVATLAS_INCLUDE_DIR})
target_include_directories(DemoUVMap PRIVATE ${DIRECTXMESH_INCLUDE_DIR})
target_include_directories(DemoUVMap PRIVATE ${WAVEFRONTREADER_INCLUDE_DIR})
target_link_libraries(DemoUVMap ${PCL_LIBRARIES})
target_link_libraries(
	DemoUVMap
	debug ${UVATLAS_LIBRARY_DEBUG}
	debug ${DIRECTXMESH_LIBRARY_DEBUG}
	optimized ${UVATLAS_LIBRARY_RELEASE}
	optimized ${DIRECTXMESH_LIBRARY_RELEASE}
)

#### DemoBlend2D
target_include_directories(DemoBlend2D PRIVATE ${BLEND2D_INCLUDE_DIR})
target_link_libraries(
	DemoBlend2D
	debug ${BLEND2D_LIBRARY_DEBUG}
	optimized ${BLEND2D_LIBRARY_RELEASE}
)

add_custom_command(TARGET DemoBlend2D POST_BUILD	# Adds a post-build event to target, copies .dll to output dir
    COMMAND ${CMAKE_COMMAND} -E copy_if_different								# executes "cmake - E copy_if_different..."
        "${CMAKE_SOURCE_DIR}/external/blend2d/build/$<CONFIG>/blend2d.dll"		# <--this is in-file
        $<TARGET_FILE_DIR:DemoBlend2D>)											# <--this is out-file path

#### DemoTexturing
target_include_directories(DemoTexturing PRIVATE ${BLEND2D_INCLUDE_DIR})
target_link_libraries(DemoTexturing ${PCL_LIBRARIES})
target_link_libraries(
	DemoTexturing
	debug ${BLEND2D_LIBRARY_DEBUG}
	optimized ${BLEND2D_LIBRARY_RELEASE}
)

add_custom_command(TARGET DemoTexturing POST_BUILD	# Adds a post-build event to target, copies .dll to output dir
    COMMAND ${CMAKE_COMMAND} -E copy_if_different								# executes "cmake - E copy_if_different..."
        "${CMAKE_SOURCE_DIR}/external/blend2d/build/$<CONFIG>/blend2d.dll"		# <--this is in-file
        $<TARGET_FILE_DIR:DemoTexturing>)										# <--this is out-file path

#### DemoVisualizer
target_link_libraries(DemoVisualizer ${PCL_LIBRARIES})

#### DemoFbx
target_include_directories(DemoFbx PRIVATE ${FBXSDK_INCLUDE_PATH})
target_link_libraries(DemoFbx ${PCL_LIBRARIES})
target_link_libraries(
	DemoFbx
	debug ${FBXSDK_LIBRARIES_DEBUG}
	optimized ${FBXSDK_LIBRARIES_RELEASE}
)

#### DemoUSD


##### pcl_visualizer_demo
#target_link_libraries(pcl_visualizer_demo ${PCL_LIBRARIES})
