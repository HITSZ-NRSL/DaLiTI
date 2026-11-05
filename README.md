# DaLiTI

## [RA-L 2025] **D**egradation-**a**ware **Li**DAR-**T**hermal-**I**nertial SLAM

### 1. Dependencies
- ROS noetic (tested)
- Basic libraries for common SLAM (e.g. openCV 4, pcl-1.10, ceres-solver-1.14 etc.)
- gtsam 4.0.3

### 2. Using Docker (Recommended)

#### 2.1 Docker Image
`run_demo.sh` builds the image on first use. Manual build:
```bash
cd YOUR_NAME_OF_WORKSPACE
docker build -t daliti:latest -f src/DaLiTI/Dockerfile .
cd src/DaLiTI && ./run_demo.sh --image daliti:latest /absolute/path/to/dataset.bag
```

#### 2.2 Run a Sequence
```bash
cd src/DaLiTI
# Indoor smoke
./run_demo.sh --profile daliti-indoor-smoke /absolute/path/to/daliti-indoor-smoke.bag
# Outdoor smoke
./run_demo.sh --profile daliti-outdoor-smoke /absolute/path/to/daliti-outdoor-smoke.bag
```
With ground truth:

```bash
./run_demo.sh --profile daliti-outdoor-smoke \
  --gt /absolute/path/to/GroundTruth_Rooftop_Smoke.tum \
  /absolute/path/to/daliti-outdoor-smoke.bag
```
Configuration files in `daliti/config/`:

| Profile | Input topic configuration | Camera calibration |
|---|---|---|
| `daliti-indoor-smoke` | `daliti-indoor-smoke_input.yaml` | `hitsz_camera.yaml` |
| `daliti-outdoor-smoke` | `daliti-outdoor-smoke_input.yaml` | `daliti-rooftop_camera.yaml` |

Run `./run_demo.sh --help` for additional options.

Results in `eskf_lio/Log/`:

```text
daliti_demo_YYYYMMDD_HHMMSS_PID.tum        trajectory
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
