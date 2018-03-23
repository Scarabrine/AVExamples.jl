# mavs_ros_planner

## status = ?

## To run
- Start Gazebo
```
roslaunch mavs_gazebo demo.launch
```
- Run RViz and load the config file from MAVs/ros/src/system/config/planner/ros_base_planner/default.rviz
- Select a 2D Pose Estimate and 2D Nav Goal in RViz. Make sure that Goal is within Global Cost map
- Base planner would create a trajectory which would be visible in RViz

## Expected Output
