Building FLANN 1.9.1 with Visual Studio
=======================================

Download
--------
1. Download FLANN 1.9.1(flann-1.9.1.zip) and unzip the file(C:\flann-1.9.1)
  <https://github.com/mariusmuja/flann/releases/tag/1.9.1>  
　

CMake
-----
1. Using the CMake-GUI tool, specify the source and build directories.   
    * **Where is the source code:**         C:\flann-1.9.1  
    * **Where is build the binaries:**      C:\flann-1.9.1\build  

2. [Configure]Click Configure and select your compiler and platform (I used Visual Studio 15 2017 / x64).     

3. Check Grouped and Advanced and verify the following settings: 
    * **BUILD_C_BINDINGS**                  ☑ (check)  
    * **BUILD_MATLAB_BINDINGS**             ☐ (uncheck)  
    * **BUILD_PYTHON_BINDINGS**             ☐ (uncheck)  
    * **CMAKE_CONFIGURATION_TYPES**         Debug;Release  
    * **CMAKE_INSTALL_PREFIX**              C:/Program Files/flann (or C:/Program Files (x86)/flann)  

4. Press [Add Entry] to add the following settings:
    <table>
      <tr>
        <td>Name:</td>
        <td>CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS</td>
      </tr>
      <tr>
        <td>Type:</td>
        <td>BOOL</td>
      </tr>
      <tr>
        <td>Value:</td>
        <td>☑(check)</td>
      </tr>
      <tr>
        <td>Description:</td>
        <td></td>
      </tr>
    </table>  
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
1. Start Visual Studio with administrator privileges and open the FLANN solution file (C:\flann-1.9.1\build\flann.sln) 
   (INSTALL will fail unless Visual Studio is started with admin privileges)  

2. Build FLANN (ALL_BUILD)  
    1. Set the solution configuration (Debug, Release).
    2. Select the ALL_BUILD project from the Solution Explorer.
    3. Build FLANN by pressing [Build]>[Build Solution].  

3. Install FLANN (INSTALL)  
    1. Select the INSTALL prject from the Solution Explorer. 
    2. Press [Build]>[Project Only]>[Build INSTALL Only] to install FLANN. Do this for both Debug / Release.   
       Necessary files are copied to the output destination specified in **CMAKE_INSTALL_PREFIX**


Environment Variable
--------------------
1. Create environment variable **FLANN_ROOT** and set it to the FLANN path (C:\Program Files\flann).  

2. Add FLANN to the environment variable **Path**.
   * ;%FLANN_ROOT%\bin 