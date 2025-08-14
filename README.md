![License](https://img.shields.io/badge/License-GPLv3-blue.svg)
![Build Status](https://img.shields.io/badge/Build-Passing-success.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Version](https://img.shields.io/badge/Version-2.0.0-blue.svg)

# ROS2 wrapper for ORB SLAM3 

Starting point of my research was working wrapper of [Mechazo11] (https://github.com/Mechazo11/ros2_orb_slam3).

Works with custom topics. Works with D455, stereo and stereo imu mode. 

![ORB-SLAM3 Stereo-Inertial Demo](imgs/Screenshot%20from%202025-08-12%2018-59-40.png)

![ORB-SLAM3 RealSense D455](imgs/Screenshot%20from%202025-08-13%2013-13-01.png)



----------------------------------------------------------------------------------

## 0. Preamble

* This package builds [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 as a shared internal library. Comes included with a number of Thirdparty libraries [DBoW2, g2o, Sophus]
* g2o used is an older version and is incompatible with the latest release found here [g2o github page](https://github.com/RainerKuemmerle/g2o).
* This package differs from other ROS1 wrappers, thien94`s ROS 1 port and ROS 2 wrappers in GitHub by supprting/adopting the following
  * A separate python node to send data to the ORB-SLAM3 cpp node. This is purely a design choice.
  * At least C++17 and Cmake>=3.8
  * Eigen 3.3.0, OpenCV 4.2, latest release of Pangolin
* Comes with a small test image sequence from EuRoC MAV dataset (MH05) to quickly test installation
* For newcomers in ROS2 ecosystem, this package serves as an example of building a shared cpp library and also a package with both cpp and python nodes.
* May not build or work correctly in **resource constrainted hardwares** such as Raspberry Pi 4, Jetson Nano

## Testing platforms

1. Intel i5-9300H, x86_64 bit architecture , Ubuntu 22.04 LTS (Jammy Jellyfish) and RO2 Humble Hawksbill (LTS)
2. AMD Ryzen 5600X, x86_64 bit architecture, Ubuntu 22.04 LTS (Jammy Jellyfish) and RO2 Humble Hawksbill (LTS)

## 1. Prerequisitis

Start with installing the following prerequisits

### Eigen3

```
sudo apt install libeigen3-dev
```

### Pangolin and configuring dynamic library path

We install Pangolin system wide and configure the dynamic library path so the necessary .so from Pangolin can be found by ros2 package during run time. More info here https://robotics.stackexchange.com/questions/105973/ros2-port-of-orb-slam3-can-copy-libdow2-so-and-libg2o-so-using-cmake-but-gettin

#### Install Pangolin

```
cd ~/Documents
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
./scripts/install_prerequisites.sh --dry-run recommended # Check what recommended softwares needs to be installed
./scripts/install_prerequisites.sh recommended # Install recommended dependencies
cmake -B build
cmake --build build -j4
sudo cmake --install build
```

#### Configure dynamic library

Check if ```/usr/lib/local``` is in the LIBRARY PATH

```bash
echo $LD_LIBRARY_PATH
```

If not, then perform the following 

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/local
sudo ldconfig
```

Then open the ```.bashrc``` file in ```\home``` directory and add these lines at the very end

```bash
if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
fi
```

Finally, source ```.bashrc``` file 

```bash
source ~/.bashrc
```
 
### OpenCV
Ubuntu 22.04 by default comes with >OpenCV 4.2. Check to make sure you have at least 4.2 installed. Run the following in a terminal

```bash
python3 -c "import cv2; print(cv2.__version__)" 
```

## 2. Installation

Follow the steps below to create the ```ros2_test``` workspace, install dependencies and build the package. Note, the workspace must be named ```ros2_test``` due to a HARDCODED path in the python node. I leave it to the developers to change this behavior as they see fit.

```bash
cd ~
mkdir -p ~/ros2_test/src
cd ~/ros2_test/src
git clone https://github.com/maikelborys/orbslam3_ros2_d455_isaacsim.git
cd .. # make sure you are in ~/ros2_ws root directory
rosdep install -r --from-paths src --ignore-src -y --rosdistro humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install

3. **Install Dependencies**:
   ```bash
   # ROS2 dependencies
   sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-tf2-ros
   
   # RealSense SDK (if using D455)
   sudo apt install ros-humble-realsense2-camera
   ```

### Build ROS2 Wrapper
```bash
cd ~/ros2_test
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_orb_slam3 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Quick Start (Odometry + TF + RViz)

This customization adds odometry publishing and optional TF broadcasting to the C++ node, plus a simple RViz flow.

### Build
```bash
cd $HOME/ros2_test
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select ros2_orb_slam3 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run (Monocular example)
Ensure a EuRoC-style dataset exists at:
`$HOME/ros2_test/src/ros2_orb_slam3/TEST_DATASET/sample_euroc_MH05/mav0/cam0/data/`

1) Start ORB‑SLAM3 C++ node (publishes odometry + TF)
```bash
cd $HOME/ros2_test && source ./install/setup.bash && \
ros2 run ros2_orb_slam3 mono_node_cpp \
  --ros-args \
  -p node_name_arg:=mono_slam_cpp \
  -p odom_topic:=/orb_slam3/odometry \
  -p odom_frame_id:=map \
  -p base_frame_id:=camera_link \
  -p publish_tf:=true
```

2) Feed images with the Python driver
```bash
cd $HOME/ros2_test && source ./install/setup.bash && \
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05
```

3) Visualize in RViz2
```bash
cd $HOME/ros2_test && source ./install/setup.bash && ros2 run rviz2 rviz2
```
- Set Fixed Frame: `map`
- Add Odometry display → Topic: `/orb_slam3/odometry`
- Optional: Add TF to see `map -> camera_link`


### Run (Stereo‑Inertial, RealSense D455) — parameterized
You can now run fully via ROS 2 parameters (no positional args), making it easy to swap topics, YAMLs, and frames.
```
cd $HOME/ros2_test && source ./install/setup.bash && \
ros2 run ros2_orb_slam3 stereo_inertial_node_cpp --ros-args \
  -p voc_file_arg:=$HOME/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin \
  -p settings_file_path_arg:=$HOME/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Stereo-Inertial/ \
  -p settings_name:=RealSense_D455 \
  -p left_image_topic:=/camera/camera/infra1/image_rect_raw \
  -p right_image_topic:=/camera/camera/infra2/image_rect_raw \
  -p imu_topic:=/camera/camera/imu \
  -p do_rectify:=false -p use_clahe:=false \
  -p odom_topic:=/orb_slam3/odometry -p odom_frame_id:=map -p base_frame_id:=camera_link -p publish_tf:=true \
  -p maps_dir:=$HOME/ros2_test/src/ros2_orb_slam3/maps
```
Notes:
- If you set `do_rectify:=true`, your YAML must provide `LEFT.*`/`RIGHT.*` rectification blocks.
- IMU initialization: set `IMU.fastInit: 1` in the YAML to bypass the “not enough acceleration” gate if starting gently.
- Performance: 640x480@30 tends to be smoother on modest CPUs.

### Map saving (Stereo‑Inertial)
- Service: `save_map` (`std_srvs/Trigger`)
- Directory: controlled by `maps_dir` parameter (default: `$HOME/ros2_test/src/ros2_orb_slam3/maps`)
- File name: `map_YYYYmmdd_HHMMSS.osa`

How to save while the node is running:
```bash
cd $HOME/ros2_test && source ./install/setup.bash
ros2 service call /save_map std_srvs/srv/Trigger {}
```
If successful, the response message shows the full path of the saved `.osa` file.

Implementation note: ORB‑SLAM3 `System::SaveMap()` writes reliably when given a simple base name (no extension) in the working directory. The node saves with a base name and then moves the resulting `.osa` into `maps_dir` to avoid path prefix issues.

## 🗺️ **Map Loading and Localization Mode**

### **Loading Pre-built Maps**
The stereo node can load previously saved maps and run in localization mode:

```bash
# Start stereo node in localization mode with loaded map
ros2 run ros2_orb_slam3 stereo_node_cpp --ros-args \
  -p voc_file_arg:=/home/robot/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin \
  -p settings_file_path_arg:=/home/robot/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Stereo/ \
  -p settings_name:=RealSense_D455 \
  -p left_image_topic:=/camera/camera/infra1/image_rect_raw \
  -p right_image_topic:=/camera/camera/infra2/image_rect_raw \
  -p odom_topic:=/orb_slam3/odometry \
  -p odom_frame_id:=map -p base_frame_id:=camera_link -p publish_tf:=true \
  -p start_localization_only:=true \
  -p load_atlas_basename:=/home/robot/ros2_test/src/ros2_orb_slam3/maps/map_20250814_003345
```

### **Reset Button Service**
Programmatically press the RESET button in the ORB-SLAM3 viewer:

```bash
# Press reset button via service
ros2 service call /press_reset_button std_srvs/srv/Trigger {}
```

**Features:**
- ✅ **Automatic Localization Mode**: Viewer automatically checks "Localization Mode" button when atlas is loaded
- ✅ **Proper State Management**: Reset service properly handles localization mode transitions
- ✅ **RViz Visualization**: Path (`/orb_slam3/path`) and map points (`/orb_slam3/markers`) for debugging
- ✅ **No Manual Button Pressing**: Fully automated localization workflow

### **Localization Workflow**
1. **Build a map** using stereo-inertial node and save it
2. **Load the map** in stereo node with `start_localization_only:=true`
3. **Localization starts automatically** - no manual button pressing needed
4. **Use reset service** if needed: `ros2 service call /press_reset_button std_srvs/srv/Trigger {}`

### **RViz Configuration**
Use the provided RViz config for visualization:
```bash
rviz2 -d ~/ros2_test/src/ros2_orb_slam3/rviz/orbslam3.rviz
```

**Available Topics:**
- `/orb_slam3/odometry` - Current pose
- `/orb_slam3/path` - Trajectory path
- `/orb_slam3/markers` - Map points and keyframes
- `/tf` - Transform tree




----------- Gracias ----------- 
```bibtex
@INPROCEEDINGS{kamal2024solving,
  author={Kamal, Azmyin Md. and Dadson, Nenyi Kweku Nkensen and Gegg, Donovan and Barbalata, Corina},
  booktitle={2024 IEEE International Conference on Advanced Intelligent Mechatronics (AIM)}, 
  title={Solving Short-Term Relocalization Problems In Monocular Keyframe Visual SLAM Using Spatial And Semantic Data}, 
  year={2024},
  volume={},
  number={},
  pages={615-622},
  keywords={Visualization;Simultaneous localization and mapping;Accuracy;Three-dimensional displays;Semantics;Robot vision systems;Pipelines},
  doi={10.1109/AIM55361.2024.10637187}}
```

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
           and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
          Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics}, 
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
 }
```

## 🚀 TODO List - Complete SLAM System Roadmap

### 🔧 **Core SLAM Features**
- [x] **Map Management**
  - [x] Map saving/loading (binary and text formats)
  - [ ] Map merging and optimization
  - [ ] Multi-session mapping support
  - [ ] Map versioning and compatibility
- [x] **Localization & Relocalization**
  - [x] Standalone localization mode
  - [x] Global relocalization from scratch
  - [ ] Loop closure detection and optimization
  - [ ] Kidnapped robot problem solver
- [ ] **Robustness Improvements**
  - [ ] Dynamic object filtering
  - [ ] Lighting condition adaptation
  - [ ] Motion blur compensation
  - [ ] Sensor fusion with wheel odometry


### 📊 **Visualization & Monitoring**
- [x] **RViz2 Integration**
  - [x] Point cloud visualization
  - [x] Trajectory plotting
  - [x] Keyframe visualization
  - [x] Map landmarks display
  - [ ] Loop closure visualization
  - [ ] Real-time performance metrics


### 🤖 **Robot Integration**
- [ ] **Navigation Stack**
  - [ ] Costmap generation from SLAM
  - [ ] NAV2 integration


### ⚙️ **Configuration & Deployment**
  - [ ] GPU acceleration support

### 📈 **Performance & Optimization**
  - [ ] GPU acceleration (CUDA)


### 🛡️ **Reliability & Safety**
- [ ] **Error Handling**
  - [ ] Graceful failure recovery
  - [ ] Automatic restart mechanisms
  - [ ] Health monitoring
- [ ] **Data Validation**
  - [ ] Input data validation
  - [ ] Configuration validation
  - [ ] Sensor data quality checks
- [ ] **Backup & Recovery**
  - [ ] Automatic map backups
  - [ ] Configuration backups
  - [ ] Recovery procedures


### 🎯 **Advanced Features**
- [ ] **Semantic SLAM**
  - [ ] Object detection integration
  - [ ] Room segmentation
  - [ ] Semantic mapping
- [ ] **Dynamic Environments**
  - [ ] Moving object tracking
  - [ ] Environment change detection
  - [ ] Adaptive mapping
- [ ] **Long-term SLAM**
  - [ ] Seasonal changes handling
  - [ ] Long-term map maintenance
  - [ ] Incremental learning

---
