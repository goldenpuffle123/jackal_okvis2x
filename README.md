# OKVIS-2X Onboard Jackal + Multiple Camera SDKs

## Environments
- ROS2 Jazzy environment is configured to work on local host

## ZED
### Setup
- Set up [ZED SDK](https://www.stereolabs.com/developers/release/latest) (system-wide)
    - For SLAM, recommended to optimize neural models - other models are irrelevant
- Set up CUDA toolkit dependency (12.8 or 13.0 recommended)
- Environment: `pixi run -e zed setup`
### Launch
- Copy zed_config/ .yaml files into zed_wrapper source (will overwrite):
    - E.g., `cp zed_config/* zed_ws/src/zed-ros2-wrapper/zed_wrapper/config`
- Enter environment
- Launch with `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 publish_imu_tf:=true`
    - Note launch file in `zed_ws/src/zed-ros2-wrapper/zed_wrapper/launch` overrides all yaml - check to be sure
### Parameters
Ensure the following topics are visible:
```bash
/zed/zed_node/depth/depth_registered             # Depth image aligned to left
/zed/zed_node/left/color/rect/image
/zed/zed_node/left/color/rect/image/camera_info  # Parameters of left
/zed/zed_node/right/color/rect/image
/zed/zed_node/right/color/rect/image/camera_info # Parameters of right
/zed/zed_node/imu/data
/tf
```
- T_SC for imu --> left: `ros2 run tf2_ros tf2_echo zed_imu_link zed_left_camera_frame_optical -p 6`
- T_SC for imu --> right: `ros2 run tf2_ros tf2_echo zed_imu_link zed_right_camera_frame_optical -p 6`
    - If not found, try `ros2 topic echo TOPIC --once --field header.frame_id`
- Parameters for left, right (look at width, height, k/p):
    - `ros2 topic echo /zed/zed_node/left/color/rect/image/camera_info --once`
    - `ros2 topic echo /zed/zed_node/right/color/rect/image/camera_info --once`

## Orbbec
### Setup
- Environment: `pixi run -e orbbec setup`
- After building, follow instructions under ROS2 SDK [build from source](https://github.com/orbbec/OrbbecSDK_ROS2#registration-script-required).
### Launch
- Enter environment
- Launch with `ros2 launch femto_config/femto_bolt.launch.py`
### Parameters
Ensure the following topics are visible:
```bash
/femtobolt/depth/image_raw    # Depth image aligned to color
/femtobolt/color/image_raw
/femtobolt/color/camera_info  # Parameters of left
/femtobolt/gyro_accel/sample  # Imu data
/tf
```
- T_SC for imu --> color: `ros2 run tf2_ros tf2_echo femtobolt_accel_gyro_optical_frame femtobolt_color_optical_frame -p 6`
    - If not found, try `ros2 topic echo TOPIC --once --field header.frame_id`
- Parameters for camera (look at width, height, k/p):
    - `ros2 topic echo /femtobolt/color/camera_info --once`
### Notes
- Imu-Depth-Color timestamp synchronization bug when stopping and restarting Orbbec SDK
    - **UNPLUG ORBBEC FROM COMPUTER AND RELAUNCH SDK**
    - **ANALYZE TIMESTAMPS** with `python tools/bag_analyze.py rosbag2`
- FOV type changes based on depth/ir width and height (see [hardware docs](https://doc.orbbec.com/documentation/Orbbec%20Femto%20Bolt%20Documentation/Femto%20Bolt%20Hardware%20Specifications))
    1. **NFOV** (640x576 / 320x288): black corners but range is ~5 m --> black corners have been confirmed working with dense mapping - use this
    2. WFOV: no black corners but range is ~2 m --> try not to use this

### Realsense
- TBD

## OKVIS-2X
### Patches
In `okvis_ws/src/OKVIS2-X/okvis_ros2/src/Subscriber.cpp` (or reference patches/Subscriber.cpp):
```diff
try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    raw = cv_ptr->image;
+       if(msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
+           raw = raw / 1000.0f; // convert to meters
+       }
} catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
}
```
```diff
-if(std::abs((tcheck - tdepth).toSec()) < OKVIS_THRESHOLD_SYNC && tcheck>=tdepth) {
+if(std::abs((tcheck - tdepth).toSec()) < OKVIS_THRESHOLD_SYNC) {
    depthSyncedTime = entry.first;
    depthImages[i] = depthImagesReceived_.at(i).at(entry.first);
    timestampedDepthImages[i] = std::make_pair(tdepth, depthImagesReceived_.at(i).at(entry.first));
    syncedDepth = true;
    break;
}
```
### Setup
```bash
pixi run -e okvis2x clone # Then confirm all submodules are cloned
# **PATCH AS ABOVE**
pixi run -e okvis2x build
```