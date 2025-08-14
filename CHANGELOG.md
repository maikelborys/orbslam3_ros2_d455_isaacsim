## Changelog

Date: 2025-08-14

### Stereo Node: Map Loading and Localization Mode
- **Map Loading**: Added support for loading pre-built maps via `load_atlas_basename` parameter
- **Localization Mode**: Added `start_localization_only` parameter to run in pure localization mode
- **Automatic Localization Button**: Fixed viewer to automatically check "Localization Mode" button when atlas is loaded
- **Reset Button Service**: Added `/press_reset_button` service (`std_srvs/Trigger`) to programmatically press the RESET button
- **Proper State Management**: Reset service properly handles localization mode by deactivating before reset and reactivating after
- **RViz Visualization**: Added path publishing (`/orb_slam3/path`) and map visualization (`/orb_slam3/markers`) for better debugging

**Implementation Details:**
- **Atlas Loading**: Uses `ORB_SLAM3_LOAD_ATLAS` environment variable and dynamic YAML patching
- **Viewer Integration**: Modified `Viewer.cc` to check system state and set localization button accordingly
- **Service Architecture**: Clean ROS2 service implementation with proper error handling and logging

**Usage:**
```bash
# Start in localization mode with loaded map
ros2 run ros2_orb_slam3 stereo_node_cpp --ros-args \
  -p start_localization_only:=true \
  -p load_atlas_basename:=/path/to/map_basename

# Press reset button programmatically
ros2 service call /press_reset_button std_srvs/srv/Trigger {}
```

### Stereo‑Inertial: Map saving service and path handling
- Added a minimal ROS 2 service `save_map` (`std_srvs/Trigger`) in `stereo_inertial_node_cpp` to persist the current ORB‑SLAM3 atlas.
- Introduced parameter `maps_dir` (default: `/home/robot/ros2_test/src/ros2_orb_slam3/maps`) to control where saved maps are stored.
- Implementation detail: ORB‑SLAM3 `System::SaveMap(<base_name>)` expects a base filename (no extension). Using absolute paths could produce an "output stream error" and log a doubled prefix (e.g., `.//home/...`). To ensure reliability, the node now:
  - Saves using a simple base name in the working directory, and
  - Moves the resulting `<base_name>.osa` into `maps_dir` atomically.
- Result: Service reliably produces timestamped files like `map_YYYYmmdd_HHMMSS.osa` under `maps_dir`.

Usage:
- While `stereo_inertial_node_cpp` runs, call:
  - `ros2 service call /save_map std_srvs/srv/Trigger {}`
- Response includes the final path of the saved `.osa` file.

Date: 2025-08-12

### Stereo‑Inertial (D455) updates
- Viewer: enabled by default in `src/stereo_inertial_main.cpp` (constructor flag=true).
- Timestamps: use full ROS time (`sec + nanosec*1e-9`) for images/IMU in `src/stereo_inertial.cpp`.
- IMU flow: collect IMU up to the image timestamp and pass `vImuMeas` into `System::TrackStereo(...)` (no core hacks).
- Odometry/TF: added `publishOdomAndTf(Twc, stamp)` with image‑timestamped `nav_msgs/Odometry` and optional TF `map -> camera_link`.
- QoS: best_effort + volatile; image depth=1 (drop stale frames), IMU depth=500.
- Latency: non‑blocking IMU collection (don’t wait for IMU to “cover” frame); removed extra sleeps; use captured left image stamp for publishing.
- Config: enabled `IMU.fastInit: 1` in `config/Stereo-Inertial/RealSense_D455.yaml` to bypass “not enough acceleration” init gate.

### New: Parameterized Stereo‑Inertial node
- Files: `include/ros2_orb_slam3/stereo_inertial.hpp`, `src/stereo_inertial.cpp`, `src/stereo_inertial_main.cpp`
- Behavior changes:
  - The node now declares/uses ROS 2 parameters instead of positional CLI args.
  - Parameters:
    - `voc_file_arg` (string, required): path to ORB vocabulary (.bin or .txt)
    - `settings_file_path_arg` (string, required): directory to Stereo‑Inertial YAMLs
    - `settings_name` (string, default `RealSense_D455`): YAML basename (adds `.yaml`)
    - `left_image_topic` (string): default `/camera/camera/infra1/image_rect_raw`
    - `right_image_topic` (string): default `/camera/camera/infra2/image_rect_raw`
    - `imu_topic` (string): default `/camera/camera/imu`
    - `do_rectify` (bool): default `false`; if true, loads `LEFT.*`/`RIGHT.*` rectification from YAML
    - `use_clahe` (bool): default `false`
    - `odom_topic` (string): default `/orb_slam3/odometry`
    - `odom_frame_id` (string): default `map`
    - `base_frame_id` (string): default `camera_link`
    - `publish_tf` (bool): default `true`
    - `min_imu_samples`, `wait_for_imu_ms`, `min_parallax_deg`, `min_tracking_points` (int/double): stability controls
- `stereo_inertial_main.cpp` simplified: constructs `ImageGrabber()` and spins; all initialization occurs in the node.
- README updated with new parameterized run command.

### Stability improvements for RealSense D455
- Updated `config/Stereo-Inertial/RealSense_D455.yaml`:
  - More conservative IMU noise parameters (increased gyro/accel noise for stability)
  - Increased ORB features (1500 vs 1200) and scale levels (8 vs 6) for better tracking
  - Improved FAST thresholds for more robust feature detection
- Added optional stability parameters to reduce map resets:
  - IMU sample requirements and timing controls
  - Minimum parallax and tracking point thresholds
  - Timestamp jump detection (disabled by default)

### Added odometry and TF publishing to C++ node
- Files:
  - `include/ros2_orb_slam3/common.hpp`
    - Added includes: `nav_msgs/msg/odometry.hpp`, `geometry_msgs/msg/transform_stamped.hpp`, `tf2_ros/transform_broadcaster.h`.
    - Added publishers/members:
      - `rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_`.
      - `std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_`.
    - Added output configuration members and defaults:
      - `odom_topic_` (default: `/orb_slam3/odometry`)
      - `odom_frame_id_` (default: `map`)
      - `base_frame_id_` (default: `camera_link`)
      - `publish_tf_` (default: `true`)
  - `src/common.cpp`
    - Declared new ROS 2 parameters: `odom_topic`, `odom_frame_id`, `base_frame_id`, `publish_tf`.
    - Read parameters and initialize `odom_publisher_` and `tf_broadcaster_`.
    - In `Img_callback(...)`:
      - Compute `Twc` from `Tcw`.
      - Publish `nav_msgs::msg::Odometry` (pose only) on `odom_topic_` with frames `odom_frame_id_` → `base_frame_id_`.
      - Optionally broadcast TF (`publish_tf_`) from `odom_frame_id_` to `base_frame_id_`.

Effect:
- New topic `/orb_slam3/odometry` provides pose for RViz.
- Optional TF `map -> camera_link` (configurable) enables RViz visualization without extra nodes.

### Build and manifest updates
- File: `CMakeLists.txt`
  - Added `find_package(nav_msgs REQUIRED)`, `find_package(geometry_msgs REQUIRED)`, `find_package(tf2_ros REQUIRED)`.
  - Added these to `THIS_PACKAGE_INCLUDE_DEPENDS` and `ament_target_dependencies` for `mono_node_cpp`.
- File: `package.xml`
  - Added build and exec depends: `nav_msgs`, `geometry_msgs`, `tf2_ros`.

Effect:
- Ensures odometry/TF symbols and message types link and are exported to downstream packages.

### Documentation
- `README.md` additions/edits:
  - Stereo‑Inertial quick start for D455 with viewer, `do_rectify=false`, and CLAHE toggle.
  - Note on `IMU.fastInit: 1` to avoid “not enough acceleration” stalls during initialization.
  - Low‑latency tips: image QoS depth=1, optionally disable CLAHE, prefer 640x480@30 if CPU bound.

### Timestamp behavior (reverted)
- Temporary change (now reverted):
  - We briefly added a fallback to use `sensor_msgs/Image.header.stamp` when the `Float64` timestep was not present.
  - At your request, this was reverted. The node again requires the `Float64` timestep on `/mono_py_driver/timestep_msg` from `mono_driver_node.py` and ignores image header stamps.

Current behavior:
- Monocular pipeline uses the Python driver to publish both the image and the `Float64` timestep. The C++ node consumes both and publishes odometry + TF.

### RealSense mono bridge + launch
- Files:
  - `scripts/realsense_mono_bridge_node.py`
    - New ROS 2 Python node that subscribes to a RealSense image topic (default `/camera/color/image_raw`) and republishes to the C++ node’s expected topics:
      - Publishes `/mono_py_driver/experiment_settings` with `settings_name` until `ACK` is received
      - Publishes `/mono_py_driver/timestep_msg` from header stamp and `/mono_py_driver/img_msg` (image passthrough)
  - `launch/realsense_orbslam3_mono.launch.py`
    - Starts `realsense2_camera_node` (color only), the bridge, and `mono_node_cpp` with odom/TF params
  - `CMakeLists.txt`
    - Installs the new script and launch directory

Effect:
- One command launch for a D455 mono feed, or run the bridge separately against an already running camera.

### Robust settings handling and segfault guard
- Files:
  - `include/ros2_orb_slam3/common.hpp`, `src/common.cpp`
    - Treat `settings_file_path_arg` as a directory to monocular YAMLs; compute full YAML path once from the received `settings_name` (e.g., `EuRoC`) → `<dir>/EuRoC.yaml`
    - Guard against double initialization (`vslam_initialized_`)
    - Initialize `pAgent` to `nullptr`; add null-check before tracking; safe shutdown in destructor
    - Wrap ORB‑SLAM3 `System` creation in try/catch and shutdown on failure

Effect:
- Fixed duplicate path bug (`EuRoC.yamlEuRoC.yaml`) and prevented re-init races, eliminating observed segfault on handshake.

### Stereo support (RealSense infra1/infra2)
- Files:
  - `include/ros2_orb_slam3/stereo.hpp`, `src/stereo.cpp`, `src/stereo_main.cpp`
    - New C++ node `stereo_node_cpp` (class `StereoMode`)
    - Sets `sensorType = ORB_SLAM3::System::STEREO`
    - Expects settings directory via `settings_file_path_arg` and a `settings_name` provided over handshake; constructs `<dir>/<settings_name>.yaml`
    - Subscribes to synchronized left/right images and a timestep, calls `TrackStereo(left, right, t)`
    - Publishes `nav_msgs/Odometry` on `/orb_slam3/odometry` and optional TF `map -> base_frame_id`
    - Guards against double initialization and null usage; safe shutdown on destruction
  - `scripts/realsense_stereo_bridge_node.py`
    - Subscribes to RealSense rectified infra topics (defaults):
      - `/camera/camera/infra1/image_rect_raw`, `/camera/camera/infra2/image_rect_raw`
    - Uses approximate time sync; publishes:
      - `/stereo_py_driver/experiment_settings` (String `settings_name`, default `RealSense_D435i`) until `ACK`
      - `/stereo_py_driver/timestep` (Float64) from header stamp
      - `/stereo_py_driver/left_img`, `/stereo_py_driver/right_img`
  - `launch/realsense_orbslam3_stereo.launch.py`
    - Starts `realsense2_camera_node` (infra1/infra2 only), the stereo bridge, and `stereo_node_cpp`
  - `CMakeLists.txt`
    - Builds/installs `stereo_node_cpp`; installs stereo bridge and launch

Effect:
- One-command stereo pipeline using D4xx infra feeds. Starts with `RealSense_D435i.yaml`; a proper `RealSense_D455.yaml` can be added subsequently (fx, fy, cx, cy, `Stereo.b`).

### How to run (Stereo — RealSense)
- One-shot launch:
```
cd /home/robot/ros2_test && source ./install/setup.bash && \
ros2 launch ros2_orb_slam3 realsense_orbslam3_stereo.launch.py \
  left_topic:=/camera/camera/infra1/image_rect_raw \
  right_topic:=/camera/camera/infra2/image_rect_raw \
  settings_name:=RealSense_D435i \
  odom_frame_id:=map base_frame_id:=camera_link publish_tf:=true \
  odom_topic:=/orb_slam3/odometry
```
- Manual:
1) C++ node
```
cd /home/robot/ros2_test && source ./install/setup.bash && \
ros2 run ros2_orb_slam3 stereo_node_cpp \
  --ros-args \
  -p settings_file_path_arg:=/home/robot/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Stereo/ \
  -p odom_topic:=/orb_slam3/odometry \
  -p odom_frame_id:=map \
  -p base_frame_id:=camera_link \
  -p publish_tf:=true
```
2) Bridge
```
chmod +x /home/robot/ros2_test/src/ros2_orb_slam3/scripts/realsense_stereo_bridge_node.py
cd /home/robot/ros2_test && source ./install/setup.bash && \
ros2 run ros2_orb_slam3 realsense_stereo_bridge_node.py \
  --ros-args \
  -p left_topic:=/camera/camera/infra1/image_rect_raw \
  -p right_topic:=/camera/camera/infra2/image_rect_raw \
  -p settings_name:=RealSense_D435i
```
3) RViz → Fixed Frame `map`; Odometry `/orb_slam3/odometry`.

Notes:
- Use 640x480 infra for best stability; ensure infra topics are rectified (`image_rect_raw`).
- Update `base_frame_id` to the desired D455 frame (e.g., `camera_infra_optical_frame`).

### How to run (RealSense mono)
1) Start the C++ node (pass settings directory):
```
cd /home/robot/ros2_test && source ./install/setup.bash && \
ros2 run ros2_orb_slam3 mono_node_cpp \
  --ros-args \
  -p settings_file_path_arg:=/home/robot/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Monocular/ \
  -p odom_topic:=/orb_slam3/odometry \
  -p odom_frame_id:=map \
  -p base_frame_id:=camera_color_optical_frame \
  -p publish_tf:=true
```
2) Start the bridge against your running camera (adjust topic if needed):
```
chmod +x /home/robot/ros2_test/src/ros2_orb_slam3/scripts/realsense_mono_bridge_node.py
cd /home/robot/ros2_test && source ./install/setup.bash && \
ros2 run ros2_orb_slam3 realsense_mono_bridge_node.py \
  --ros-args -p input_image_topic:=/camera/camera/color/image_raw \
             -p settings_name:=EuRoC
```
3) RViz:
```
cd /home/robot/ros2_test && source ./install/setup.bash && ros2 run rviz2 rviz2
```
- Fixed Frame `map`; Odometry topic `/orb_slam3/odometry`.

Notes:
- If RealSense resolution differs greatly from the YAML (e.g., 1280x720 vs EuRoC 752x480), prefer 640x480 color for stability. Set via RealSense params or `realsense2_camera` CLI.
- Ensure only one bridge instance runs to avoid duplicate handshakes.

### Run commands (reference)
1) C++ node
```
cd /home/robot/ros2_test && source ./install/setup.bash && ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp -p odom_topic:=/orb_slam3/odometry -p odom_frame_id:=map -p base_frame_id:=camera_link -p publish_tf:=true
```
2) Python driver (EuRoC sample)
```
cd /home/robot/ros2_test && source ./install/setup.bash && ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05
```
3) RViz2
```
cd /home/robot/ros2_test && source ./install/setup.bash && ros2 run rviz2 rviz2
```
RViz: Fixed Frame `map`, Odometry topic `/orb_slam3/odometry`.



### Stereo (D455) YAML corrections and usage
- Files:
  - `orb_slam3/config/Stereo/RealSense_D455.yaml`
    - Updated to the schema ORB‑SLAM3 stereo expects:
      - `Camera.type: "PinHole"`
      - Rectified intrinsics: `Camera.fx/fy/cx/cy`
      - Zero distortion for rectified streams: `Camera.k1/k2/p1/p2 = 0.0`
      - Baseline term: `Camera.bf = baseline[m] * fx` (D455 ≈ 0.095 m)
      - Resolution/fps to match RealSense profile
    - Alternative: include `Tlr` (left→right) if needed. Keep `Camera.bf` defined regardless.
- Run (stereo, no IMU):
  - Start RealSense with infra only (rectified topics) and then `stereo_node_cpp` with `settings_name:=RealSense_D455`.
- Notes:
  - Streams must be rectified (`.../image_rect_raw`).
  - If you see device errors or dropped frames, prefer 640x480@30 for stability.
  - The D435i stereo config from the reference package also works if you adjust `Camera.bf` and intrinsics to your device.
