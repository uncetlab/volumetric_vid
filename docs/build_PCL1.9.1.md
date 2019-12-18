Building PCL 1.9.1 with Visual Studio
=====================================

Download  
--------
1. Get PCL Sources. (C:\PCL-1.9.1)  
  <https://github.com/PointCloudLibrary/pcl/tree/pcl-1.9.1>  


3rdParty  
--------
1. Build and install the 3rd party libs:
    * Boost 1.68.0 [build_Boost1.68.0.md](build_Boost1.68.0.md)  
    * Eigen 3.3.5 [build_Eigen3.3.5.md](build_Eigen3.3.5.md)
    * FLANN 1.9.1 [build_FLANN1.9.1.md](build_FLANN1.9.1.md) 
    * QHull 2015.2 [build_QHull2015.2.md](build_QHull2015.2.md)
    * VTK 8.1.2 [build_VTK8.1.2.md](build_VTK8.1.2.md)
    * OpenNI 2.2.0.33 Beta <http://structure.io/openni>

2. Set Environment variables for the 3rd party libs: 
    * **BOOST_ROOT**                        C:\Program Files\Boost  
    * **EIGEN_ROOT**                        C:\Program Files\Eigen3  
    * **FLANN_ROOT**                        C:\Program Files\flann  
    * **QHULL_ROOT**                        C:\Program Files\qfull
    * **VTK_DIR**                           C:\Program Files\VTK  

CMake  
-----
1. Using the CMake-GUI tool, specify the source and build directories.
    * **Where is the source code:**         C:\PCL-1.9.1  
    * **Where is build the binaries:**      C:\PCL-1.9.1\build  

2. [Configure] Click Configure and select your compiler and platform (I used Visual Studio 15 2017 / x64). 

3. Check Grouped and Advanced and verify the following settings: 
    **Ungrouped Entries**  
    * **EIGEN_INCLUDE_DIR**                 C:\Program Files\Eigen3\include\eigen3  
    * **VTK_DIR**                           C:\Program Files\VTK\lib\cmake\vtk-8.1  

    **BUILD**  
    * **BUILD_2d**                          ☑(check)  
    * **BUILD_CUDA**                        ☐(uncheck)  
    * **BUILD_GPU**                         ☐(uncheck)  
    * **BUILD\_all\_in\_one\_installer**    ☑(check)<sup>*1</sup>     
    * **BUILD_apps**                        ☐(uncheck)  
    * **BUILD_common**                      ☑(check)  
    * **BUILD_example**                     ☐(uncheck)  
    * **BUILD_features**                    ☑(check)  
    * **BUILD_filters**                     ☑(check)  
    * **BUILD_geometry**                    ☑(check)   
    * **BUILD\_global\_tests**              ☐(uncheck)  
    * **BUILD_io**                          ☑(check)   
    * **BUILD_kdtree**                      ☑(check)   
    * **BUILD_keypoints**                   ☑(check)  
    * **BUILD_ml**                          ☑(check)  
    * **BUILD_octree**                      ☑(check)   
    * **BUILD_outofcore**                   ☑(check)   
    * **BUILD_people**                      ☑(check)   
    * **BUILD_recognition**                 ☑(check)   
    * **BUILD_registration**                ☑(check)   
    * **BUILD\_sample\_consensus**          ☑(check)   
    * **BUILD_search**                      ☑(check)   
    * **BUILD_segmentation**                ☑(check)   
    * **BUILD_simulation**                  ☐(uncheck)   
    * **BUILD_stereo**                      ☑(check)   
    * **BUILD_surface**                     ☑(check)   
    * **BUILD\_surface\_on\_nurbs**         ☑(check)   
    * **BUILD_tools**                       ☑(check)<sup>*2</sup>   
    * **BUILD_tracking**                    ☑(check)   
    * **BUILD_visualization**               ☑(check)   

   <sup> \*1 When the INSTALL project is built, PCL and the 3rd party libs are installed to \<CMAKE\_INSTALL\_PREFIX\>(C:\Program Files\PCL)</sup>  
   <sup> \*2 Since it takes time to build, disable option if you don't require the various tools (bin\\pcl\_\*.exe) </sup>  

    Verify these paths are correct on your system, some may require a 'x64' in the filename.

    **Boost**  
    * **Boost_ATOMIC_LIBRARY_DEBUG**          C:\Program Files\Boost\lib\libboost_atomic-vc141-mt-gd-1_68.lib  
    * **Boost_ATOMIC_LIBRARY_RELEASE**        C:\Program Files\Boost\lib\libboost_atomic-vc141-mt-1_68.lib  
    * **Boost_CHRONO_LIBRARY_DEBUG**          C:\Program Files\Boost\lib\libboost_chrono-vc141-mt-gd-1_68.lib  
    * **Boost_CHRONO_LIBRARY_RELEASE**        C:\Program Files\Boost\lib\libboost_chrono-vc141-mt-1_68.lib  
    * **Boost_DATE_TIME_LIBRARY_DEBUG**       C:\Program Files\Boost\lib\libboost_date_time-vc141-mt-gd-1_68.lib  
    * **Boost_DATE_TIME_LIBRARY_RELEASE**     C:\Program Files\Boost\lib\libboost_date_time-vc141-mt-1_68.lib  
    * **Boost_FILESYSTEM_LIBRARY_DEBUG**      C:\Program Files\Boost\lib\libboost_filesystem-vc141-mt-gd-1_68.lib  
    * **Boost_FILESYSTEM_LIBRARY_RELEASE**    C:\Program Files\Boost\lib\libboost_filesystem-vc141-mt-1_68.lib 
    * **Boost_INCLUDE_DIR**                   C:\Program Files\Boost\include\boost-1_68
    * **Boost_IOSTREAMS_LIBRARY_DEBUG**       C:\Program Files\Boost\lib\libboost_iostreams-vc141-mt-gd-1_68.lib  
    * **Boost_IOSTREAMS_LIBRARY_RELEASE**     C:\Program Files\Boost\lib\libboost_iostreams-vc141-mt-1_68.lib  
    * **Boost_LIBRARY_DIR_DEBUG**             C:\Program Files\Boost\lib  
    * **Boost_LIBRARY_DIR_RELEASE**           C:\Program Files\Boost\lib  
    * **Boost_MPI_LIBRARY_DEBUG**             C:\Program Files\Boost\lib\libboost_mpi-vc141-mt-gd-1_68.lib  
    * **Boost_MPI_LIBRARY_RELEASE**           C:\Program Files\Boost\lib\libboost_mpi-vc141-mt-1_68.lib  
    * **Boost_REGEX_LIBRARY_DEBUG**           C:\Program Files\Boost\lib\libboost_regex-vc141-mt-gd-1_68.lib  
    * **Boost_REGEX_LIBRARY_RELEASE**         C:\Program Files\Boost\lib\libboost_regex-vc141-mt-1_68.lib  
    * **Boost_SERIALIZATION_LIBRARY_DEBUG**   C:\Program Files\Boost\lib\libboost_serialization-vc141-mt-gd-1_68.lib  
    * **Boost_SERIALIZATION_LIBRARY_RELEASE** C:\Program Files\Boost\lib\libboost_serialization-vc141-mt-1_68.lib 
    * **Boost_SYSTEM_LIBRARY_DEBUG**          C:\Program Files\Boost\lib\libboost_system-vc141-mt-gd-1_68.lib  
    * **Boost_SYSTEM_LIBRARY_RELEASE**        C:\Program Files\Boost\lib\libboost_system-vc141-mt-1_68.lib  
    * **Boost_THREAD_LIBRARY_DEBUG**          C:\Program Files\Boost\lib\libboost_thread-vc141-mt-gd-1_68.lib  
    * **Boost_THREAD_LIBRARY_RELEASE**        C:\Program Files\Boost\lib\libboost_thread-vc141-mt-1_68.lib  

    **CMAKE**  
    * **CMAKE_CONFIGURATION_TYPES**         Debug;Release  
    * **CMAKE_INSTALL_PREFIX**              C:\Program Files\PCL  

    **FLANN**  
    * **FLANN_INCLUDE_DIR**                 C:\Program Files\flann\include  
    * **FLANN_LIBRARY**                     C:\Program Files\flann\lib\flann_cpp_s.lib  
    * **FLANN_LIBRARY_DEBUG**               C:\Program Files\flann\lib\flann_cpp_s-gd.lib  

    **OPENNI2**  
    * **OPENNI2_INCLUDE_DIRS**              C:\Program Files\OpenNI2\Include  
    * **OPENNI2_LIBRARY**                   C:\Program Files\OpenNI2\Lib\OpenNI2  

    **QHULL**  
    * **QHULL_INCLUDE_DIR**                 C:\Program Files\qhull\include  
    * **QHULL_LIBRARY**                     C:\Program Files\qhull\lib\qhullstatic.lib  
    * **QHULL_LIBRARY_DEBUG**               C:\Program Files\qhull\lib\qhullstatic_d.lib  

    **WITH**  
    * **WITH_CUDA**                         ☐(uncheck)  
    * **WITH_DAVIDSDK**                     ☐(uncheck)  
    * **WITH_DOCS**                         ☐(uncheck)  
    * **WITH_DSSDK**                        ☐(uncheck)  
    * **WITH_ENSENSO**                      ☐(uncheck)  
    * **WITH_FZAPI**                        ☐(uncheck)  
    * **WITH_LIBUSB**                       ☐(uncheck)  
    * **WITH_OPENGL**                       ☑(check)  
    * **WITH_OPENNI**                       ☐(uncheck)  
    * **WITH_OPENNI2**                      ☑(check)  
    * **WITH_PCAP**                         ☐(uncheck)  
    * **WITH_PNG**                          ☐(uncheck)  
    * **WITH_QHULL**                        ☑(check)  
    * **WITH_QT**                           ☐(uncheck)  
    * **WITH_RSSDK**                        ☐(uncheck)  
    * **WITH_VTK**                          ☑(check)  

4. Check the output to confirm the packages were found. 
    * Boost version: 1.68.0  
      Found the following Boost libraries:  
        system  
        filesystem  
        thread  
        date_time  
        iostreams  
        chrono  
        atomic  
        regex  
    * Eigen found (include: C:/Program Files/Eigen/include/eigen3, version: 3.3.5)  
    * FLANN found (include: C:/Program Files/flann/include, lib: optimized;C:/Program Files/flann/lib/flann\_cpp\_s.lib;debug;C:/Program Files/flann/lib/flann\_cpp\_s-gd.lib)  
    * QHULL found (include: C:/Program Files/qhull/include, lib: optimized;C:/Program Files/qhull/lib/qhullstatic.lib;debug;C:/Program Files/qhull/lib/qhullstatic_d.lib)  
    * VTK\_MAJOR\_VERSION 8, rendering backend: OpenGL  
VTK found (include: C:/Program Files/VTK/include/vtk-8.1, lib: vtkChartsCore;vtkCommonColor;vtkCommonCore;vtksys;vtkCommonDataModel;vtkCommonMath;vtkCommonMisc;vtkCommonSystem;vtkCommonTransforms;vtkCommonExecutionModel;vtkFiltersGeneral;vtkCommonComputationalGeometry;vtkFiltersCore;vtkInfovisCore;vtkFiltersExtraction;vtkFiltersStatistics;vtkImagingFourier;vtkImagingCore;vtkalglib;vtkRenderingContext2D;vtkRenderingCore;vtkFiltersGeometry;vtkFiltersSources;vtkRenderingFreeType;vtkfreetype;vtkzlib;vtkDICOMParser;vtkDomainsChemistry;vtkIOLegacy;vtkIOCore;vtklz4;vtkIOXMLParser;vtkexpat;vtkFiltersAMR;vtkIOXML;vtkParallelCore;vtkFiltersFlowPaths;vtkFiltersGeneric;vtkFiltersHybrid;vtkImagingSources;vtkFiltersHyperTree;vtkFiltersImaging;vtkImagingGeneral;vtkFiltersModeling;vtkFiltersParallel;vtkFiltersParallelImaging;vtkFiltersPoints;vtkFiltersProgrammable;vtkFiltersSMP;vtkFiltersSelection;vtkFiltersTexture;vtkFiltersTopology;vtkFiltersVerdict;verdict;vtkGeovisCore;vtkIOImage;vtkmetaio;vtkjpeg;vtkpng;vtktiff;vtkInfovisLayout;vtkImagingHybrid;vtkInteractionStyle;vtkInteractionWidgets;vtkImagingColor;vtkRenderingAnnotation;vtkRenderingVolume;vtkViewsCore;vtkproj4;vtkIOAMR;vtkhdf5_hl;vtkhdf5;vtkIOEnSight;vtkIOExodus;vtkexoIIc;vtkNetCDF;vtkIOExport;vtkRenderingGL2PS;vtkRenderingContextOpenGL;vtkRenderingOpenGL;vtkgl2ps;vtklibharu;vtkIOExportOpenGL;vtkRenderingLabel;vtkIOGeometry;vtkIOImport;vtkIOInfovis;vtklibxml2;vtkIOLSDyna;vtkIOMINC;vtkIOMovie;vtkoggtheora;vtkIONetCDF;vtknetcdfcpp;vtkIOPLY;vtkIOParallel;vtkjsoncpp;vtkIOParallelXML;vtkIOSQL;vtksqlite;vtkIOTecplotTable;vtkIOVideo;vtkImagingMath;vtkImagingMorphological;vtkImagingStatistics;vtkImagingStencil;vtkInteractionImage;vtkRenderingImage;vtkRenderingLIC;vtkRenderingLOD;vtkRenderingVolumeOpenGL;vtkViewsContext2D;vtkViewsInfovis  
    * OpenNI 2 found (include: C:/Program Files/OpenNI2/Include, lib: C:/Program Files/OpenNI2/Lib/OpenNI2.lib, redist: C:\Program Files\OpenNI2\Redist\)  

5. [Generate] Click Generate to output the .sln file. 

Build  
-----
1. Start Visual Studio with administrator privileges and open the PCL solution file (C:\PCL-1.9.1\build\PCL.sln). (INSTALL will fail unless Visual Studio is started with admin privileges)

2. Build PCL (ALL_BUILD)  
    1. Set the solution configuration (Debug, Release).
    2. Select the ALL_BUILD project from the Solution Explorer.
    3. Build PCL by pressing [Build]>[Build Solution].

3. Install PCL (INSTALL)  
    1. Select the INSTALL prject from the Solution Explorer. 
    2. Press [Build]>[Project Only]>[Build INSTALL Only] to install PCL. Do this for both Debug / Release.   
       Necessary files are copied to the output destination specified in **CMAKE_INSTALL_PREFIX**


Environment Variable  
--------------------
1. Create environment variable **PCL_ROOT** and set it to the PCL path (C:\Program Files\PCL).

2. Add the PCL and 3rdParty paths to the environment variable **Path**. 
    * ;%PCL_ROOT%\bin  
    * ;%OPENNI2_REDIST64%  