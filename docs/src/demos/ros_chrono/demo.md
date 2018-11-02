# ros_chrono

A vehicle model in `Chrono` that can be used through `ROS`.

## Status == working
The vehicle model currently runs with rigid tire models, a rear-wheel driveline, double wishbone suspension (reduced so that the control arm positions are distance constraints), and rack and pinion steering.

## To run

```
$ cd $HOME/MAVs/ros
$ roslaunch ros_chrono demo.launch
$ rosparam set system/default/flags/initialized true
```

## Expected Output
![link](../images/chrono_demo/expectedoutputchronodemo.png)


## To run velocity test
Velocity test shows the ability of chorono to exchange information with ros. 
```
$ roslaunch ros_chrono demo_velocity_control.launch
```
## Expected Output
![link](../images/chrono_demo/demo_velocity_control.png)
