# demoC

## vehicle_description and ros_base_planner

A demo that shows `ros_base_planner` calculating a path from the LiDAR data collected from the `vehicle_description` package.

### status = working
I do not see the path that is being planned displayed in RViz

## To Run
```
roslaunch system demoC.launch
```

## Expected Output
Gazebo(with the vehicle) and Rviz(showing LIDAR scan) would pop up.

`Case1`: 'system/ros_base_planner/flags/goal_known' in 'MAVs/ros/src/system/config/system/demoC.yaml' is set to `false`. User can click on publish goal button in Rviz and select a goal point within the gloabl cost map area. The planner would plan a path from start to goal and display it.
It will also update following ros parameters with trajectory data: `vehicle/chrono/ros_base_planner/traj/x`, `vehicle/chrono/ros_base_planner/traj/y`.

`Case2`: 'system/ros_base_planner/flags/goal_known' is set to `true`: Planner would pick the goal coordinates from case.yaml file and plot a trajectory on Rviz and will update the ros parameters as mentioned above.
Note: goal point has to be within the global cost map for the planner to ba able to plan.

After, the trajectory is published, ros parameter `/system/ros_base_planner/initialized` will be set to `true`
This is not closed loop, it just shows the connectivity of these nodes within a system.
