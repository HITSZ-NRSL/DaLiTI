# DaLiTI

## [RA-L 2025] **D**egradation-**a**ware **Li**DAR-**T**hermal-**I**nertial SLAM


This repository contains code for a Degradation aware LiDAR-Thermal-Inertial SLAM, which is accepted by IEEE RA-L, June 2025.


What changed in this version: [CHANGES.md](CHANGES.md).

### 1. Dependency
- ROS noetic (tested)
- Basic libraries for common SLAM (e.g. openCV 4, pcl-1.10, ceres-solver-1.14 etc.)
- gtsam 4.0.3

### 2. Using Docker (Recommended)

#### 2.1 Docker Image
`run_demo.sh` builds the image automatically the first time it runs (tag `daliti:<git commit>`, built from the
committed code; this takes several minutes and needs internet access; set `http_proxy` / `https_proxy` if your network
requires a proxy). To build an image by hand from the workspace root and use it:
```bash
cd YOUR_NAME_OF_WORKSPACE
docker build -t daliti:latest -f src/DaLiTI/Dockerfile .
cd src/DaLiTI && ./run_demo.sh --image daliti:latest /absolute/path/to/dataset.bag
```

#### 2.2 Run a Sequence
```bash
cd src/DaLiTI
# indoor smoke sequence (whole bag)
./run_demo.sh /absolute/path/to/daliti-indoor-smoke.bag
# rooftop smoke sequence (90 s .. 177 s, back near the start point; rooftop camera calibration), evaluated against the ground truth
./run_demo.sh --profile rooftop --gt /absolute/path/to/GroundTruth_Rooftop_Smoke.tum /absolute/path/to/rooftop.bag
```
`./run_demo.sh --help` lists the options: `--profile`, `--start` / `--duration` (playback window), `--camera NAME`
(`daliti/config/NAME_camera.yaml`), `--image-topic`, `--gt`, `--image`, `--rebuild`.

The script starts the container with RViz (and NVIDIA graphics if available), waits for all nodes, plays the bag (or
the selected window), stops the container, checks that the trajectory covers the window, and evaluates it. Each run
writes to `eskf_lio/Log/`:

```text
daliti_demo_YYYYMMDD_HHMMSS_PID.tum        poses, stamped with the time of the state (end of each LiDAR scan)
daliti_demo_YYYYMMDD_HHMMSS_PID.log        console log
daliti_demo_YYYYMMDD_HHMMSS_PID_eval.json  path length, end-to-start distance, ATE (with --gt)
```

### 3. Run Examples (Local Installation)
```
mkdir -p YOUR_NAME_OF_WORKSPACE/src
cd YOUR_NAME_OF_WORKSPACE/src
git clone https://github.com/HITSZ-NRSL/DaLiTI.git
cd ..
```
`catkin_make` or `catkin build`

```
roslaunch daliti demo.launch rosbag_file_with_path:="YOUR/PATH/TO/ROSBAG/XXX.bag"
```
For validation, the hitsz degration dataset is provided: [dowload link](https://entuedu-my.sharepoint.com/:f:/g/personal/yufeng004_e_ntu_edu_sg/EpL7PLCR5p1OrfxFnaIS1-oBOHiVjv-PG6_BFTvu3ANp9Q?e=kkutbE) 

### 4.  Acknowledgements
DaLiTI is partly modified codes form [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM), [FAST-LIO2](https://github.com/hku-mars/FAST_LIO), [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono), [ESKF_LIO](https://github.com/chengwei0427/ESKF_LIO.git) and [rebvo](https://github.com/JuanTarrio/rebvo). Thanks for the contributers.  

Contributer: [Yufeng Liu](https://github.com/raymond-lau-lyf)

### 5.  Citation
If you use our work, please cite:
```
@ARTICLE{11045071,
         author={Wang, Yu and Liu, Yufeng and Chen, Lingxu and Chen, Haoyao and Zhang, Shiwu},
         journal={IEEE Robotics and Automation Letters},
         title={Degradation-Aware LiDAR-Thermal-Inertial SLAM},
         year={2025},
         volume={10},
         number={8},
         pages={8035-8042},
         doi={10.1109/LRA.2025.3581127}}
```
## LICENSE
The source code is released under GPLv3 license.
