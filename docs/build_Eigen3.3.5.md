Building Eigen 3.3.5 with Visual Studio
=======================================

Download
--------
1. Download Eigen 3.3.5(eigen-eigen-b3f3d4950030.zip) and unzip the file. (C:\eigen-eigen-b3f3d4950030)  
   <http://eigen.tuxfamily.org/index.php?title=News:Eigen_3.3.5_released!>  
   <https://bitbucket.org/eigen/eigen/downloads/>  


CMake
-----
1. Using the CMake-GUI tool, specify the source and build directories.  
    * **Where is the source code:**         C:\eigen-eigen-b3f3d4950030  
    * **Where is build the binaries:**      C:\eigen-eigen-b3f3d4950030\build  

2. [Configure] Click Configure and select your compiler and platform (I used Visual Studio 15 2017 / x64).   

3. Check Grouped and Advanced and verify the following settings: 
    * **CMAKE_INSTALL_PREFIX**              C:\Program Files\Eigen3 (or C:\Program Files (x86)\Eigen3)  

4. [Generate] Click Generate to output the .sln file.  


Build
-----
1. Start Visual Studio with administrator privileges and open the Eigen solution file (C:\eigen-eigen-b3f3d4950030\build\Eigen.sln). (INSTALL will fail unless Visual Studio is started with admin privileges)

2. Install Eigen. (INSTALL)
    1. Select the INSTALL prject from the Solution Explorer. 
    2. Press [Build]>[Project Only]>[Build INSTALL Only] to install PCL. Do this for both Debug / Release.   
       Necessary files are copied to the output destination specified in **CMAKE_INSTALL_PREFIX** 


Environment Variable
--------------------
1. Create environment variable **EIGEN_ROOT** and set it to the Eigen path (C:\Program Files\Eigen3).