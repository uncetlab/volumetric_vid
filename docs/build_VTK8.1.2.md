Building VTK 8.1.2 with Visual Studio
=====================================

Download
--------
1. Download VTK 8.1.2 (VTK-8.1.2.zip) and unzip the file (C:\VTK-8.1.2)
  <http://www.vtk.org/VTK/resources/software.html#latestcand>  
  <https://github.com/Kitware/VTK/tree/v8.1.2>  
　

CMake
-----
1. Using the CMake-GUI tool, specify the source and build directories.
    * **Where is the source code:**         C:\VTK-8.1.2
    * **Where is build the binaries:**      C:\VTK-8.1.2\build

2. [Configure] Click Configure and select your compiler and platform (I used Visual Studio 15 2017 / x64).   

3. Check Grouped and Advanced and verify the following settings:
    **BUILD**  
    * **BUILD_SHAREED_LIBS**                ☐ (uncheck)
    * **BUILD_TESTING**                     ☐ (uncheck)

    **CMAKE**  
    * **CMAKE_CONFIGURATION_TYPES**         Debug;Release  
    * **CMAKE_CXX_MP_FLAG**                 ☑ (check)   
    * **CMAKE_INSTALL_PREFIX**              C:/Program Files/VTK (or C:/Program Files (x86)/VTK)  

    **VTK**  
    * **VTK_RENDERING_BACKEND**             OpenGL <sup>*1</sup>  

    <sup>*1 Temporary measures to avoid this problem ([#1601](https://github.com/PointCloudLibrary/pcl/issues/1601)).</sup>  

4. Press [Add Entry] to add the following settings:  
    <table>
      <tr>
        <td>Name:</td>
        <td>CMAKE_DEBUG_POSTFIX*</td>
      </tr>
      <tr>
        <td>Type:</td>
        <td>STRING</td>
      </tr>
      <tr>
        <td>Value:</td>
        <td>-gd</td>
      </tr>
      <tr>
        <td>Description:</td>
        <td></td>
      </tr>
    </table>  
    <sup>* A character string to be added to the file name (tail) of the generated file of Debug build</sup>  

5. [Generate] Click Generate to output the .sln file.


Build
-----
1. Start Visual Studio with administrator privileges and open the VTK solution file (C:\VTK-8.1.2\build\VTK.sln)
   (INSTALL will fail unless Visual Studio is started with admin privileges) 

2. Build VTK (ALL_BUILD)  
    1. Set the solution configuration (Debug, Release).
    2. Select the ALL_BUILD project from the Solution Explorer.
    3. Build VTK by pressing [Build]>[Build Solution].  

3. Install VTK (INSTALL)  
    1. Select the INSTALL prject from the Solution Explorer. 
    2. Press [Build]>[Project Only]>[Build INSTALL Only] to install VTK. Do this for both Debug / Release.   
       Necessary files are copied to the output destination specified in **CMAKE_INSTALL_PREFIX**


Environment Variable
--------------------
1. Create environment variable **VTK_DIR** and set it to the VTK path (C:\Program Files\VTK)