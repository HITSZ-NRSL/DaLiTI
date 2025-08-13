# DaLiTI

## [RA-L 2025] **D**egradation-**a**ware **Li**DAR-**T**hermal-**I**nertial SLAM


This repository contains code for a Degradation aware LiDAR-Thermal-Inertial SLAM, which is accepted by IEEE RA-L, June 2025.


### 1. Dependency
- ROS noetic (tested)
- Basic libraries for common SLAM (e.g. openCV 4, pcl-1.10, etc.)
- gtsam 4.0.3

### 2. Run Examples
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
For validation, the hitsz degration dataset is provided: [dowload link](https://drive.google.com/drive/folders/1U44TfwOejWUmdcbuG6x2k14r6DkX6V-k) 

### 3.  Acknowledgements
DaLiTI is partly modified codes form [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM), [FAST-LIO2](https://github.com/hku-mars/FAST_LIO), [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono), [ESKF_LIO](https://github.com/chengwei0427/ESKF_LIO.git) and [rebvo](https://github.com/JuanTarrio/rebvo). Thanks for the contributers.  

Contributer: [Yufeng Liu](https://github.com/raymond-lau-lyf)

### 4.  Citation
If you use our work, please cite:
```
@ARTICLE{AURO-D-21-00050R1,
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
