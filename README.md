# OKVIS-2X Onboard Jackal + Multiple Camera SDKs

## Environments
- ROS2 Jazzy environment is configured to work on local host

## ZED
### Setup
- Set up [ZED SDK](https://www.stereolabs.com/developers/release/latest) (system-wide)
    - For SLAM, recommended to optimize neural models - other models are irrelevant
- Set up CUDA toolkit dependency (12.8 or 13.0 recommended)
    - 12.8 for best native OKVIS2-X compatibility
    - 13.0 for best native cuVSLAM compatibility
- Environment: `pixi run -e zed setup`
### Launch
- Copy zed_config/ .yaml files into zed_wrapper source (will overwrite):
    - E.g., `cp zed_config/* zed_ws/src/zed-ros2-wrapper/zed_wrapper/config`
- Enter environment
- Launch with `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2`

## Orbbec
### Setup
- Environment: `pixi run -e orbbec setup`
- After building, following instructions under ROS2 SDK [build from source](https://github.com/orbbec/OrbbecSDK_ROS2#registration-script-required).
### Launch
- Enter environment
- Launch with `ros2 launch femto_config/femto_bolt.launch.py`

### Realsense
- TBD