# demoI

## `Obstacle_detector` is used to detect the obstacle information and pass it to `nloptcontrol_planner`. Then the generated trajectory will be passed to chrono vehicle model, and chrono vehicle model will follow this trajectory. 

## status = working??
There are still previous obstacle information in the current frame.  

## To Run
```
roslaunch system demoI.launch
```

## Expected Output
![link](demoI/demoI_2.png)
![link](demoI/demoI.png)

Frame `map` is not published until `nloptcontrol_planner` is initialized.
