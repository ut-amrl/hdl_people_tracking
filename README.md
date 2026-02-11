# hdl_people_tracking (ROS 2)

This is a ROS 2 port of the `hdl_people_tracking` package.

## Build

```bash
cd ~/nav_ws
colcon build --packages-select hdl_people_tracking
source install/setup.bash
```

## Usage

Launch the people tracking nodes:

```bash
ros2 launch hdl_people_tracking hdl_people_tracking.launch.py
```

Arguments:
- `static_sensor` (default: `false`): Set to `true` if the sensor is fixed (not moving).

### Input Topics
- `/velodyne_points` (`sensor_msgs/msg/PointCloud2`): Input 3D point cloud.
- `/odom` (`nav_msgs/msg/Odometry`): Robot odometry (required if `static_sensor` is `false`).

### Output Topics
- `clusters` (`hdl_people_tracking/msg/ClusterArray`): Detected clusters.
- `tracks` (`hdl_people_tracking/msg/TrackArray`): Tracked people.
- `markers` (`visualization_msgs/msg/MarkerArray`): Visualization markers for tracks.
- `detection_markers` (`visualization_msgs/msg/MarkerArray`): Visualization markers for detections.
