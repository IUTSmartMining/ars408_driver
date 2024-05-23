# Continental ARS408 Driver

This is Continental ARS408 driver for ROS2 which reads data from **ethernet interface**.

## How to use

1. install dependencies

```sh
rosdep install --from-paths src --ignore-src -r -y
```

2. build

```sh
colcon build
```

3. source package

```sh
source install/setup.bash
```

4. launch the driver

```sh
ros2 launch launch/continental_ars408.launch.xml
```

## Design

### Interface arguments

To create a TCP server and bind it to this address.

- `ip_address` (Default: `192.168.100.9`)
- `port` (Default: `50000`)

### Input

- `input/frame`
  - `can_msgs` <https://github.com/ros-industrial/ros_canopen/tree/melodic-devel/can_msgs>

### Output

- `output/objects`
  - `RadarTrack`: <https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTrack.msg>
  - If you want to visualize, you should choose `RadarTrack` and visualize in rviz using [radar_tracks_msgs_converter](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/radar_tracks_msgs_converter) with autoware.universe.
- `output/return`
  - `RadarReturn`: <https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarReturn.msg>

### Parameters

- `publish_radar_track`
  - The bool parameter to publish `output/objects` topic
- `publish_radar_return`
  - The bool parameter to publish `output/return` topic
- `output_frame`
  - The string parameter of the output frame id
- `sequential_publish`
  - The bool parameter to determine output publishing behavior.
  - If this parameter is set to false (default value), the driver will publish output after receiving a complete cycle of sequential data from the CAN data topic.
  - If this parameter is set to true, the driver will publish output every time data is received from the CAN data topic.
- `size_x`
  - The assumed x-axis size of output objects [m]. The default parameter is 1.8, which derive from distance resolution measuring of ARS408 for far range.
- `size_y`
  - The assumed y-axis size of output objects [m]. The default parameter is 1.8, which derive from distance resolution measuring of ARS408 for far range.

## Reference

- This repository fork from original package [Perception Engine's Continental ARS408 Driver](https://gitlab.com/perceptionengine/pe-drivers/ars408_ros)
