# Dream Gmapping

Launch full simulation with gamepad, gazebo, rviz:
```bash
source src/dream_mobile_platform/dream_feature_flags
SIM=true roslaunch dream_mobile_platform low_level_drivers.launch
```
- This internally calls [this launch file](https://github.com/RicoJia/The-Dream-Robot/blob/master/src/dream_base_description/README.md#L9) from `dream_base_description`

To launch `dream_odometer` and `dream_gmapper`:
```bash
roslaunch dream_gmapping dream_gmapping.launch sim:=true
```
## Tools

- Record Bags

```bash
rosbag record --bz2 /dream/scan /dream/wheel_pos /dream_gazebo_client/map_groud_truth_pose /tf/ /tf_static
```

- Replay bags: `rosbag play --clock -r <RATE_IN_HZ> <BAG_NAME>`

## Assumptions

1. Since the number of lidar points could vary from brand to brand, model to model, and even from frequency to frequency,
   it is determined dynamically when the first laser scan is received.
