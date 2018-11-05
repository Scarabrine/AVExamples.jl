# Developer

## Modifying files
### Modifying files that do not need to be compiled
There are several file types that can be modified without having to recompile:
 YAML files: configuration files
Launch files: these files orchestrate the launching the nodes and scripts needed to run the demos
Julia and Python files: these files contain the scripts needed to accomplish certain tasks

### Modifying files that need to be compiled (i.e. C++ files)
You can rebuild the workspace in two ways. The first method takes shorter time.

Rebuild the ros workspace inside the container:  
```
$cd ~/MAVs/ros
$catkin_make
```
After that, you need to commit the change of image:  
Open a new terminal
```
$docker ps -l
$docker commit <container_name> <image_name>
```

      2. Rebuild the image if container is terminated:
```
$sh build.sh
```
## Creating a demo
TODO

## Testing
TODO

## Documentation
TODO

## Making a pull request
TODO
