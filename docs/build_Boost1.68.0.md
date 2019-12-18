Building Boost 1.68.0 with Visual Studio
========================================

Download
--------
1. Download Boost 1.68.0 (boost_1_68_0.zip) and unzip the file(C:\boost_1_68_0)  
   <http://www.boost.org/users/history/version_1_68_0.html>  
   <https://github.com/boostorg/boost/tree/boost-1.68.0>  

2. [optional (i skipped-mh-12.18.19)] Microsoft MPI v9.0.1(MSMpiSetup.exe)とMicrosoft MPI SDK v9.0.1(msmpisdk.msi)をダウンロードしてインストールする。  
   (それぞれC:\Program Files\Microsoft MPIとC:\Program Files (x86)\Microsoft SDKs\MPIにインストールされる。)  
   <https://www.microsoft.com/en-us/download/details.aspx?id=56727>  

Build
-----
1. [optional (i skipped-mh-12.18.19)] Merge the following commit fixes:  
    * Don't break other toolsets that inherit from msvc (such as clang-win). Refs #330. | boostorg/build  
      <https://github.com/boostorg/build/commit/d11e3e4ef342b7723b3cf554fde7677873c47040>

2. Start the Visual Studio Developer Command Prompt ("Developer Command Prompt for VS20XX" or "x86/x64 Native Tools Commond Prompt for VS20XX") with administrator privileges and execute the following command.
   <sup>* if the developer command prompt is not started with admin privileges, install will fail</sup>    
    * Win32  
      ```
      cd C:\boost_1_68_0
      bootstrap.bat
      ```

    * x64  
      ```
      cd C:\boost_1_68_0
      bootstrap.bat
      ```

3. [optional (i skipped-mh-12.18.19)] Modify the generated project-config.jam settings.  
    * boost_1_68_0\project-config.jam  
      4 line (add)  
      ```
      using mpi ;
      ```

4. Run the following command in the Visual Studio Developer command prompt.
    <sup>※ The -j option specifies the number of parallel compilations. Usually specifies the number of CPU logical processors.</sup>   
    <sup>※ Specify "11.0" for 2012, "12.0" for 2013, "14.0" for 2015, and "14.1" for 2017.</sup>   
    * Win32  
      ```
      b2.exe toolset=msvc-14.1 address-model=32 --build-dir=build\x86 install --prefix="C:\Program Files (x86)\Boost" -j8
      ```
      
    * x64  
      ```
      b2.exe toolset=msvc-14.1 address-model=64 --build-dir=build\x64 install --prefix="C:\Program Files\Boost" -j8
      ```  

5. If the build is successful, the following files are generated in <prefix> / lib. 
    * libboost_atomic-vc141-mt-1_68.lib  
    * libboost_atomic-vc141-mt-gd-1_68.lib  
    * libboost_chrono-vc141-mt-1_68.lib  
    * libboost_chrono-vc141-mt-gd-1_68.lib  
    * libboost_container-vc141-mt-1_68.lib  
    * libboost_container-vc141-mt-gd-1_68.lib  
    * libboost_context-vc141-mt-1_68.lib  
    * libboost_context-vc141-mt-gd-1_68.lib  
    * libboost_coroutine-vc141-mt-1_68.lib  
    * libboost_coroutine-vc141-mt-gd-1_68.lib  
    * libboost_date_time-vc141-mt-1_68.lib  
    * libboost_date_time-vc141-mt-gd-1_68.lib  
    * libboost_exception-vc141-mt-1_68.lib  
    * libboost_exception-vc141-mt-gd-1_68.lib  
    * libboost_fiber-vc141-mt-1_68.lib  
    * libboost_fiber-vc141-mt-gd-1_68.lib  
    * libboost_filesystem-vc141-mt-1_68.lib  
    * libboost_filesystem-vc141-mt-gd-1_68.lib  
    * libboost_graph-vc141-mt-1_68.lib  
    * libboost_graph-vc141-mt-gd-1_68.lib  
    * libboost_graph_parallel-vc141-mt-1_68.lib  
    * libboost_graph_parallel-vc141-mt-gd-1_68.lib  
    * libboost_iostreams-vc141-mt-1_68.lib  
    * libboost_iostreams-vc141-mt-gd-1_68.lib  
    * libboost_locale-vc141-mt-1_68.lib  
    * libboost_locale-vc141-mt-gd-1_68.lib  
    * libboost_log-vc141-mt-1_68.lib  
    * libboost_log-vc141-mt-gd-1_68.lib  
    * libboost_log_setup-vc141-mt-1_68.lib  
    * libboost_log_setup-vc141-mt-gd-1_68.lib  
    * libboost_math_c99-vc141-mt-1_68.lib  
    * libboost_math_c99-vc141-mt-gd-1_68.lib  
    * libboost_math_c99f-vc141-mt-1_68.lib  
    * libboost_math_c99f-vc141-mt-gd-1_68.lib  
    * libboost_math_c99l-vc141-mt-1_68.lib  
    * libboost_math_c99l-vc141-mt-gd-1_68.lib  
    * libboost_math_tr1-vc141-mt-1_68.lib  
    * libboost_math_tr1-vc141-mt-gd-1_68.lib  
    * libboost_math_tr1f-vc141-mt-1_68.lib  
    * libboost_math_tr1f-vc141-mt-gd-1_68.lib  
    * libboost_math_tr1l-vc141-mt-1_68.lib  
    * libboost_math_tr1l-vc141-mt-gd-1_68.lib  
    * libboost_mpi-vc141-mt-1_68.lib  
    * libboost_mpi-vc141-mt-gd-1_68.lib  
    * libboost_prg_exec_monitor-vc141-mt-1_68.lib  
    * libboost_prg_exec_monitor-vc141-mt-gd-1_68.lib  
    * libboost_program_options-vc141-mt-1_68.lib  
    * libboost_program_options-vc141-mt-gd-1_68.lib  
    * libboost_random-vc141-mt-1_68.lib  
    * libboost_random-vc141-mt-gd-1_68.lib  
    * libboost_regex-vc141-mt-1_68.lib  
    * libboost_regex-vc141-mt-gd-1_68.lib  
    * libboost_serialization-vc141-mt-1_68.lib  
    * libboost_serialization-vc141-mt-gd-1_68.lib  
    * libboost_signals-vc141-mt-1_68.lib  
    * libboost_signals-vc141-mt-gd-1_68.lib  
    * libboost_system-vc141-mt-1_68.lib  
    * libboost_system-vc141-mt-gd-1_68.lib  
    * libboost_test_exec_monitor-vc141-mt-1_68.lib  
    * libboost_test_exec_monitor-vc141-mt-gd-1_68.lib  
    * libboost_thread-vc141-mt-1_68.lib  
    * libboost_thread-vc141-mt-gd-1_68.lib  
    * libboost_timer-vc141-mt-1_68.lib  
    * libboost_timer-vc141-mt-gd-1_68.lib  
    * libboost_type_erasure-vc141-mt-1_68.lib  
    * libboost_type_erasure-vc141-mt-gd-1_68.lib  
    * libboost_unit_test_framework-vc141-mt-1_68.lib  
    * libboost_unit_test_framework-vc141-mt-gd-1_68.lib  
    * libboost_wave-vc141-mt-1_68.lib  
    * libboost_wave-vc141-mt-gd-1_68.lib  
    * libboost_wserialization-vc141-mt-1_68.lib  
    * libboost_wserialization-vc141-mt-gd-1_68.lib  

[optional (i skipped-mh-12.18.19)] : Build Boost.Python and Boost.NumPy
---------------------------------------
1. Python 3.7.1(python-3.7.1.exeまたはpython-3.7.1-amd64.exe)をダウンロードしてインストールする。(C:\Python37)  
   カスタムインストールを選択、以下の設定でインストールする。  
   <https://www.python.org/downloads/release/python-371/>  
   
    **Advanced Options**  
    * **Install for all users**               ☑(check)  
    * **Add Python to environment variables** ☑(check)  

    * **Customize install location**          C:\Python37\

2. NumPy 1.15.4(numpy‑1.15.4+mkl‑cp37‑cp37m‑win32.whlまたはnumpy‑1.15.4+mkl‑cp37‑cp37m‑win_amd64.whl)のWheelをダウンロードする。(C:\\)  
   コマンドプロンプトで以下のコマンドを実行する。  
   <http://www.lfd.uci.edu/~gohlke/pythonlibs/#numpy>     
    * Win32  
    ```
    cd C:\
    pip install numpy‑1.15.4+mkl‑cp37‑cp37m‑win32.whl
    ```
   
    * x64
    ```
    cd C:\
    pip install numpy‑1.15.4+mkl‑cp37‑cp37m-win_amd64.whl
    ```

3. project-config.jamの設定を修正する。  
    <sup>※ Visual C++のバージョン(14.1)とコンパイラ(cl.exe)を指定する。</sup>  
    <sup>※ Boost.MPIと同時にビルドできないため、設定(using mpi ; )をコメントアウトする。</sup>    
    <sup>※ Pythonのバージョン(3.7)、ディレクトリ、インクルードディレクトリ、ライブラリディレクトリ、アドレスモデルを指定する。</sup>    
    * Win32  
      3-7 line (fix and add)  
      ```
      using msvc : 14.1 : "C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\VC\Tools\MSVC\14.15.26726\bin\HostX64\x86\cl.exe" ; 
      
      # using mpi ;
      
      using python : 3.7 : C:\\Python37 : C:\\Python37\\include : C:\\Python37\\libs : ; 
      ```  
    
    * x64  
      3-7 line (fix and add)  
      ```  
      using msvc : 14.1 : "C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\VC\Tools\MSVC\14.15.26726\bin\HostX64\x64\cl.exe" ; 
      
      # using mpi ;
      
      using python : 3.7 : C:\\Python37 : C:\\Python37\\include : C:\\Python37\\libs : <address-model>64 ; 
      ```   
  
4. Visual Studioの開発者コマンドプロンプト("開発者コマンドプロンプト for VS20XX"または"VS20XX x86/x64 Native Tools Commond Prompt")に続けて以下のコマンドを実行する。  
    <sup>※ -jオプションはコンパイルの並列数を指定する。通常はCPUの論理プロセッサ数を指定する。</sup>   
    <sup>※ toolsetはVisual Studioのバージョンを指定する。2012は「11.0」、2013は「12.0」、2015は「14.0」、2017は「14.1」を指定する。</sup>   
    * Win32  
      ```
      b2.exe toolset=msvc-14.1 address-model=32 --build-dir=build\x86 install --prefix="C:\Program Files (x86)\Boost" -j8
      ```
      
    * x64  
      ```
      b2.exe toolset=msvc-14.1 address-model=64 --build-dir=build\x64 install --prefix="C:\Program Files\Boost" -j8
      ```  

5. ビルドが成功すると\<prefix\>/libに以下のファイルが生成される。  
    * libboost_numpy3-vc141-mt-1_68.lib  
    * libboost_numpy3-vc141-mt-gd-1_68.lib  
    * libboost_python3-vc141-mt-1_68.lib
    * libboost_python3-vc141-mt-gd-1_68.lib

[optional (i skipped-mh-12.18.19)] : Build Boost.bzip2 and Boost.zlib
-----------------------------------------
1. bzip2(bzip2-1.0.6.tar.gz)とzlib(zlib1211.zip)をダウンロードして展開する。(C:\\)  
   <http://www.bzip.org/downloads.html>  
   <http://zlib.net/>    

2. Visual Studioの開発者コマンドプロンプト("開発者コマンドプロンプト for VS20XX"または"VS20XX x86/x64 Native Tools Commond Prompt")に続けて以下のコマンドを実行する。  
    <sup>※ -jオプションはコンパイルの並列数を指定する。通常はCPUの論理プロセッサ数を指定する。</sup>   
    <sup>※ toolsetはVisual Studioのバージョンを指定する。2012は「11.0」、2013は「12.0」、2015は「14.0」、2017は「14.1」を指定する。</sup>   
    * Win32  
      ```
      b2.exe toolset=msvc-14.1 address-model=32 --build-dir=build\x86 install --prefix="C:\Program Files (x86)\Boost" -j8 -s BZIP2_SOURCE="C:\bzip2-1.0.6" -s ZLIB_SOURCE="C:\zlib-1.2.11" 
      ```
      
    * x64  
      ```
      b2.exe toolset=msvc-14.1 address-model=64 --build-dir=build\x64 install --prefix="C:\Program Files\Boost" -j8 -s BZIP2_SOURCE="C:\bzip2-1.0.6" -s ZLIB_SOURCE="C:\zlib-1.2.11"
      ```  
5. ビルドが成功すると\<prefix\>/libに以下のファイルが生成される。  
    * libboost_bzip2-vc141-mt-1_68.lib  
    * libboost_bzip2-vc141-mt-gd-1_68.lib  
    * libboost_zlib-vc141-mt-1_68.lib  
    * libboost_zlib-vc141-mt-gd-1_68.lib  

Environment Variable
--------------------
1. Create environment variable **BOOST_ROOT** and set the PCL path (C:\Program Files\Boost).