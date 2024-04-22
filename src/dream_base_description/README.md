# Dream Base Description

This project is an educational project where I became familiar with visualization using rviz, and simulation of a differential drive on Gazebo.

Here you will find where this simulation fits into the entire dream_robot_platform system: ![Workflow Depiction](../dream_mobile_platform/README.md).

## Usage

- To bring up the gazebo simulation and the Rviz simulation

```
roslaunch dream_base_description dream_base_sim.launch
```

- To skip rviz and only keep the gazebo simulation:

```
roslaunch dream_base_description dream_base_sim.launch no_rviz:=true
```

- In the Gazebo simulation, we publish `/map_ground_truth -> /base_link` on TF as the ground truth. Meanwhile, it publishes onto `wheel_joint_pos_topic`, and listens to `commanded_wheel_vel_topic` for wheel velocity commands (see [the config file for example](./config/diff_params.yaml))
