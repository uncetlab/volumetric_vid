Building the demo data

Requirements:
 - ffmpeg: https://ffmpeg.org/
 - matlab with Computer Vision Toolbox add-on

Building:
 - open a bash console, make sure ffmpeg is added to PATH
 - clone the CMU Panoptic dataset toolbox (somewhere outside of volumetric_vid repo, like `/d/repos`):
   - `git clone git@github.com:CMU-Perceptual-Computing-Lab/panoptic-toolbox.git`
 - clone our fork (with normal estimation) of the CMU Panoptic dataset toolbox (somewhere outside of volumetric_vid repo, like `/d/repos`):
   - `git clone git@github.com:mhudnell/panoptic-toolbox.git`
 - download a sample sequence of kinect data:
   - `cd panoptic-toolbox`
   - `./scripts/getData_kinoptic.sh 171026_pose3`
 - extract the RGB frames:
   - `cd 171026_pose3`
   - `../scripts/kinectImgsExtractor.sh`
 - generate pt clouds:
   - edit `./matlab/demo_kinoptic_gen_ptcloud.m` line 25:
        ```
        root_path = 'C:\path\to\panoptic-toolbox'
        seqName = '171026_pose3'
        hd_index_list= 380:500;
        ```
   - `matlab ../matlab/demo_kinoptic_gen_ptcloud.m`
 - ?
   - `matlab ./matlab/demo_kinoptic_projection.m`
