# vehicle_description and ros_base_planner

A demo that shows ``ros_base_planner`` calculating a path from the LiDAR data collected from the ``vehicle_description`` package.

### status = needs fix
I do not see the path that is being planned displayed in RViz

## To Run
```
roslaunch system demoC.launch
```

## Expected Output
Gazebo should pop up and if you move the view so that you can see to the right ``(x,y)=(0,200)``, you will see the vehicle. All of the nodes are thin initialized and the ``nloptcontrol_planner`` node takes the longest, so for a few minutes the terminal screen will display
```
waiting on obstacle_avoidance.jl in nloptcontrol_planner ...
waiting on obstacle_avoidance.jl in nloptcontrol_planner ...
waiting on obstacle_avoidance.jl in nloptcontrol_planner ...
waiting on obstacle_avoidance.jl in nloptcontrol_planner ...
```
Eventually, the controller will be ready and the vehicle will start to move every time a new solution is generated. This is not closed loop, it just shows the connectivity of these nodes within a system.
