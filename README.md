# DaLiTI

## [RA-L 2025] **D**egradation-**a**ware **Li**DAR-**T**hermal-**I**nertial SLAM


This repository contains code for a Degradation aware LiDAR-Thermal-Inertial SLAM, which is accepted by IEEE RA-L, June 2025.


### 1. Dependency
- ROS noetic (tested)
- Basic libraries for common SLAM (e.g. openCV 4, pcl-1.10, ceres-solver-1.14 etc.)
- gtsam 4.0.3

### 2. Using Docker (Recommended)

#### 2.1 Build Docker Image
First, navigate to your workspace and build the Docker image:
```bash
cd YOUR_NAME_OF_WORKSPACE
docker build -t daliti:latest -f src/DaLiTI/Dockerfile .
```

#### 2.2 Run Docker Container
Use the provided startup script to launch the container with RViz support:
```bash
cd src/DaLiTI
./start_docker.sh
```

The script will:
- Mount the source code directory (`src/DaLiTI`) into the container
- Enable X11 forwarding for RViz visualization
- Start an interactive zsh shell with ROS environment pre-configured

#### 2.3 Run Examples in Docker
Once inside the container, you can run the demo:
```bash
roslaunch daliti demo.launch rosbag_file_with_path:="/ws/YOUR/PATH/TO/ROSBAG/XXX.bag"
```

**Note:** Make sure to place your rosbag files in a location accessible to the container, or mount additional directories when starting the container.

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
