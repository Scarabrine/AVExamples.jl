# data_logging

## System logger

This demonstrates how we can use rosbag record using a launch file. Details for existing rosbag APIs can be found here [rosbag](http://wiki.ros.org/rosbag/Commandline). For implementation details in launch file, please refer to [Record with rosbag from launch file](https://answers.ros.org/question/52773/record-with-rosbag-from-launch-file/). We record complete `tf` tree being pulished and filter out the tranformation between `map` and `base_footprint` using a shell script.

## Steps

### 1. Add similar snippet to your launch file
Put following code in your demo launch file:

```xml
<?xml version="1.0"?>
<launch>
  <arg name="system_params_path" default="$(find system)/config/system/demos/demoD.yaml"/>

  <!-- Add your nodes -->

  <node pkg="rosbag" type="record" name="record"
       args="record -O robot_tf.bag tf"/>

  <node name="bootstrap" pkg="system" type="bootstrap.jl" output="screen"/>

</launch>
```
Here we specified the topics to record as `args` to `record` node, in this case `tf` topic will be saved in a file called `robot_tf.bag` in `<home>/.ros/`

### 2. Run follwing in cmd prompt
```
$(rospack find system)/scripts/rosbag_filter.sh
```
This script will create a `$system_pkg_path/data/robot_tf.txt` which is a csv file containing tranformation between `map` and `base_footprint` that can be used for plotting vehicle path
