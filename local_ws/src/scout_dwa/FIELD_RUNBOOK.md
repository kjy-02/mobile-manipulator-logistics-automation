# Scout DWA Field Runbook

> **과거 자료:** AMSL DWA 이식 후의 현재 구성과 실행 방법은 `README.md`와
> `IMPLEMENTATION_STATUS.md`를 따른다. 이 문서의 기존 실차 명령은 현재
> 설정값 검증과 별도 운용 지시 없이 실행하지 않는다.

## Goal

Use the static map below for DWA obstacle checking and drive through waypoints:

```bash
/home/rilab/catkin_ws/src/hdl_graph_slam/map.pcd
```

Current waypoint command:

```bash
roslaunch scout_dwa dwa_map.launch waypoints:="16.217,-4.194;-1.852,8.286;-30.167,-22.737"
```

## Modified Files

Only files inside `scout_dwa` were changed.

- `src/dwa_navigator.cpp`
  - Loads `map.pcd` directly through `~map_pcd_path`.
  - Downsamples the PCD map and builds a KD-tree.
  - Extracts nearby map points around the robot and transforms them from `map` frame into the robot local frame for DWA obstacle scoring.
  - Uses `/map -> /base_link`, with fallback to `/map -> /velodyne`.
  - Supports `waypoints` parameter in this format:

    ```text
    x1,y1;x2,y2;x3,y3
    ```

  - Switches to the next waypoint after reaching the current one.
  - Stops after reaching the final waypoint.
  - New RViz `/move_base_simple/goal` clears waypoint mode and uses that single goal.

- `launch/dwa_map.launch`
  - Default DWA map:

    ```bash
    /home/rilab/catkin_ws/src/hdl_graph_slam/map.pcd
    ```

  - Adds `waypoints`, `goal_x`, `goal_y`, `cmd_vel_topic`, `map_frame`, `base_frame`, `local_map_radius`, and `map_voxel_leaf_size` args.
  - `goal_x` and `goal_y` default to empty, so the robot does not drive to `(0,0)` unless a goal/waypoints are explicitly provided.

- `launch/hdl_localization_map.launch`
  - Optional localization launch that uses the same map as DWA:

    ```bash
    /home/rilab/catkin_ws/src/hdl_graph_slam/map.pcd
    ```

  - `specify_init_pose=false`.
  - Intended to be initialized from RViz with `2D Pose Estimate`.

- `CMakeLists.txt`
  - Compile standard changed to C++14 because PCL 1.10 headers require it.

## Important Map Note

The original localization launch does not use the same map:

```text
hdl_localization.launch:
  /home/rilab/catkin_ws/src/hdl_localization/data/map/hard.pcd

dwa_map.launch:
  /home/rilab/catkin_ws/src/hdl_graph_slam/map.pcd
```

For the DWA run described here, use:

```bash
roslaunch scout_dwa hdl_localization_map.launch
```

instead of:

```bash
roslaunch hdl_localization hdl_localization.launch
```

## Field Startup Sequence

Open one terminal per block and keep each process running.

### Terminal 0: roscore

```bash
source ~/catkin_ws/devel/setup.bash
roscore
```

### Terminal 1: CAN setup

```bash
source ~/catkin_ws/devel/setup.bash
rosrun scout_bringup bringup_can2usb.bash
```

### Terminal 2: Scout base

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch scout_bringup scout_robot_base.launch
```

### Terminal 3: Velodyne

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch velodyne_pointcloud VLP16_points.launch
```

Check:

```bash
rostopic hz /velodyne_points
```

### Terminal 4: Localization with the DWA map

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch scout_dwa hdl_localization_map.launch
```

### Terminal 5: Velodyne to base_link TF

```bash
source ~/catkin_ws/devel/setup.bash
rosrun tf_code velodyne_to_baselink.py
```

### Terminal 6: RViz

```bash
source ~/catkin_ws/devel/setup.bash
roscd hdl_localization/rviz
rviz -d hdl_localization.rviz
```

In RViz:

1. Set `Fixed Frame` to `map`.
2. Add `PointCloud2` for `/globalmap` if needed.
3. Select `2D Pose Estimate`.
4. Click the robot's current location on the map and drag in the robot's facing direction.

### TF Check

After RViz pose initialization:

```bash
rosrun tf tf_echo map base_link
```

If that does not work:

```bash
rosrun tf tf_echo map velodyne
```

If coordinates continue updating, localization is ready.

### Run DWA

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch scout_dwa dwa_map.launch waypoints:="16.217,-4.194;-1.852,8.286;-30.167,-22.737"
```

Keep the `waypoints:=...` argument on the same line and quote it because it contains semicolons.

## Emergency Stop

Stop the DWA launch with `Ctrl+C`.

If the robot still receives velocity commands, publish zero velocity:

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{}'
```

## Coordinate Notes

- Waypoints are `map` frame `x,y` meter coordinates.
- Use RViz `Publish Point` and read:

  ```bash
  rostopic echo /clicked_point
  ```

- Use RViz `2D Nav Goal` and read:

  ```bash
  rostopic echo /move_base_simple/goal
  ```

- Localization initialization uses `2D Pose Estimate`, not `2D Nav Goal`.

## Last Verified

`dwa_navigator` was built successfully with:

```bash
cmake --build /home/rilab/catkin_ws/build --target dwa_navigator -j2
```
