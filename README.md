### Installation
  * PCL 1.9.1: https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.9.1
    * download installer: `PCL-1.9.1-AllInOne-msvc2017-win64.exe`
      * install to default `C:\Program Files\PCL 1.9.1`
    * download pdbs: `pcl-1.9.1-pdb-msvc2017-win64.zip`
      * contains debug info
      * extract to `C:\Program Files\PCL 1.9.1\bin`
    * download sources:
      * helpful when debugging
      * extract to `C:\Program Files\PCL 1.9.1\`
      * when you attempt to step in to some PCL code for the first time it will ask for you to direct it to the source code
    * note: if you want to be able to make changes to PCL source, refer to sec3 on building PCL from source
  * UVAtlas
    * in `volumetric_vid/external`:
    * `git clone --depth=1 https://github.com/microsoft/UVAtlas.git`
    * open `UVAtlas_2017_Win10.sln` in Visual Studio 2017
    * `UVAtlas_2017_Win10`>Build (in Release x64 / Debug x64)
  * DirectXMesh
    * in `volumetric_vid/external`:
    * `git clone --depth=1 https://github.com/microsoft/DirectXMesh.git`
    * open `DirectXMesh_Desktop_2017_Win10.sln` in Visual Studio 2017
    * `DirectXMesh`>Build (in Release x64 / Debug x64)
  * Blend2D
    * in `volumetric_vid/external`:
      ```bash
      git clone --depth=1 https://github.com/asmjit/asmjit
      git clone --depth=1 https://github.com/blend2d/blend2d
      ```
  * FBX SDK 2020.1
    * download: https://www.autodesk.com/developer-network/platform-technologies/fbx-sdk-2020-0 [VS 2017]
    * install to default location `C:\Program Files`
    * download [optional]: [FBX Review](https://www.autodesk.com/products/fbx/fbx-review) to view fbx files
  * USD SDK
    * clone and run build script (requires python2 & pyside): (run from a `x64 Native Tools Command Prompt for VS2017`)
      ```bash
      git clone https://github.com/PixarAnimationStudios/USD.git
      conda create --name python2
      conda activate python2
      conda install python=2.7
      conda install pip
      conda install pyopengl
      conda install jinja2
      pip install pyside  # install via pip bc not available in conda
      python USD\build_scripts\build_usd.py "C:\Program Files\USD"  # adding --debug breaks python (won't be able to run in Debug mode, but Release / RelWithDebInfo work)
      # python USD\build_scripts\build_usd.py "C:\Program Files\USD-debug" --debug --no-python
      ```
      * build in debug configuration separately to "C:\Program Files\USD-debug" if you wish to debug
    * add .dll locations to path (e.g. `C:\Program Files\USD\lib` & `C:\Program Files\USD\bin`)
    * if using the usd executables (e.g. `usdview`), you must run the command from a bash shell, it doesn't work in normal windows terminal

### Building doxygen documentation
  * You can generate html and latex documentation (requires installing [doxygen](http://www.doxygen.nl/download.html#srcbin)): 
    * via gui:
      1. Open doxygen gui `doxywizard.exe`
      2. "Specify the working dir from which doxygen will run" (folder with Doxyfile): `<path-to-project>\docs\doxygen`
      3. Run
    * via cmd:
      ```bash
      cd docs/doxygen
      doxygen Doxyfile
      ```

### Build PCL (and it's dependencies) from sources
  * Refer to the following documentation for building PCL (and it's dependencies) from sources with Visual Studio on Windows: [docs/build_PCL1.9.1.md](docs/build_PCL1.9.1.md) 
    * Note the above documentation was adapted / translated from the following source: <https://gist.github.com/UnaNancyOwen/59319050d53c137ca8f3#file-pcl1-9-1-md>
