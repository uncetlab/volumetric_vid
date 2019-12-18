Building QHull 2015.2 with Visual Studio
========================================

Download
--------
1. Download QHull 2015.2 for Windows(qhull-2015.2.zip)and unzip the file (C:\qhull-2015.2)  
   <http://www.qhull.org/download/>  


CMake  
-----
0. Delete the existing build folder. (C:\qhull-2015.2\build)  

1. Using the CMake-GUI tool, specify the source and build directories.     
    * **Where is the source code:**         C:\qhull-2015.2  
    * **Where is build the binaries:**      C:\qhull-2015.2\build  

2. [Configure] Click Configure and select your compiler and platform (I used Visual Studio 15 2017 / x64).   

3. Check Grouped and Advanced and verify the following settings:   
    * **CMAKE_CONFIGURATION_TYPES**         Debug;Release  
    * **CMAKE_INSTALL_PREFIX**              C:\Program Files\qhull (or C:\Program Files (x86)\qhull)  

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
        <td>_d</td>
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
1. Start Visual Studio with administrator privileges and open the QHull solution file (C:\qhull-2015.2\build\qhull.sln)
   (INSTALL will fail unless Visual Studio is started with admin privileges)  

2. Build QHull (ALL_BUILD)  
    1. Set the solution configuration (Debug, Release).
    2. Select the ALL_BUILD project from the Solution Explorer.
    3. Build QHull by pressing [Build]>[Build Solution].    

3. Install QHull (INSTALL)  
    1. Select the INSTALL prject from the Solution Explorer. 
    2. Press [Build]>[Project Only]>[Build INSTALL Only] to install QHull. Do this for both Debug / Release.   
       Necessary files are copied to the output destination specified in **CMAKE_INSTALL_PREFIX**


Environment Variable
--------------------
1. Create environment variable **QHULL_ROOT** and set it to the QHull path (C:\Program Files\qhull)