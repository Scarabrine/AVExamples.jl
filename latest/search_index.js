var documenterSearchIndex = {"docs": [

{
    "location": "index.html#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": ""
},

{
    "location": "index.html#MAVs-Documentation-1",
    "page": "Home",
    "title": "MAVs Documentation",
    "category": "section",
    "text": ""
},

{
    "location": "index.html#Introduction-1",
    "page": "Home",
    "title": "Introduction",
    "category": "section",
    "text": "This software simulates autonomous vehicles within a ROS environment."
},

{
    "location": "index.html#Installation-1",
    "page": "Home",
    "title": "Installation",
    "category": "section",
    "text": ""
},

{
    "location": "index.html#Docker-1",
    "page": "Home",
    "title": "Docker",
    "category": "section",
    "text": ""
},

{
    "location": "index.html#Local-1",
    "page": "Home",
    "title": "Local",
    "category": "section",
    "text": "dependencies:sudo apt-get install ros-kinetic-move-base"
},

{
    "location": "index.html#Citation-1",
    "page": "Home",
    "title": "Citation",
    "category": "section",
    "text": "If you find NLOptControl.jl useful, please cite it:@software{nlopt,\n  author = {{Huckleberry Febbo}},\n  title = {NLOptControl.jl},\n  url = {https://github.com/JuliaMPC/NLOptControl.jl},\n  version = {0.0.1},\n  date = {2017-06-17},\n}If you find VehicleModels.jl useful, please cite this paper:@Conference{Febbo2017,\n  author    = {Huckleberry Febbo, Jiechao Liu, Paramsothy Jayakumar, Jeffrey L. Stein, Tulga Ersal},\n  title     = {Moving Obstacle Avoidance for Large, High-Speed Autonomous Ground Vehicles},\n  year      = {2017},\n  publisher = {IEEE}\n}"
},

{
    "location": "index.html#Acknowledgements-1",
    "page": "Home",
    "title": "Acknowledgements",
    "category": "section",
    "text": ""
},

{
    "location": "packages/nloptcontrol_planner/index.html#",
    "page": "nloptcontrol_planner",
    "title": "nloptcontrol_planner",
    "category": "page",
    "text": ""
},

{
    "location": "packages/nloptcontrol_planner/index.html#nloptcontrol_planner-1",
    "page": "nloptcontrol_planner",
    "title": "nloptcontrol_planner",
    "category": "section",
    "text": "Obstacle avoidance algorithm described in this paper which can now be solved in real time using an official julia package called NLOptControl.jl.If a vehicle needs to drive quickly and avoid moving obstacles then NLOptControl.jl is well suited to solve the trajectory planning problem. Where A* and OpenPlanner are path planning algorithms and are mostly concerned with satisfying kinematic/geometric constraints, they can identify a path to follow, but without a temporal component, they do not tell you how to follow the path. While, trajectory planning also considers how you can follow the path. So, for instance, what are the optimal steering and throttle trajectories (not simply what are the X and Y waypoints).NLOptControl.jl is designed as a high level tool, so researchers can easily define their own optimization problems, see my documentation here.To do this, nloptcontrol_planner uses the MAVs.jl package."
},

{
    "location": "packages/nloptcontrol_planner/index.html#MAVs.jl-1",
    "page": "nloptcontrol_planner",
    "title": "MAVs.jl",
    "category": "section",
    "text": "MAVs.jl is a julia package that solves an autonomous vehicle motion planning problem using NLOptControl.jl.There are several different modules, nothing is documented. Read the source code for more info."
},

{
    "location": "packages/nloptcontrol_planner/index.html#Flags-and-Settings-1",
    "page": "nloptcontrol_planner",
    "title": "Flags and Settings",
    "category": "section",
    "text": "Name Description\n/nloptcontrol_planner/case_name name of configuration file for solver settings\n/nloptcontrol_planner/obstacle_name name of configuration file for obstacle field\n/nloptcontrol_planner/flags/3DOF_plant indicates if the 3DOF plant model in VehicleModels.jl will be used\n/nloptcontrol_planner/flags/init indicates if the planner has finished initilization\n/nloptcontrol_planner/flags/known_environment indicates if the obstacle information is assumed to be knownParameters are broken into two categories; Outputs and Inputs. In the demo, the inputs are also generated, but flags can be set to let the node know that the user will be setting these rosparams externally."
},

{
    "location": "packages/nloptcontrol_planner/index.html#Inputs-1",
    "page": "nloptcontrol_planner",
    "title": "Inputs",
    "category": "section",
    "text": ""
},

{
    "location": "packages/nloptcontrol_planner/index.html#Obstacles-1",
    "page": "nloptcontrol_planner",
    "title": "Obstacles",
    "category": "section",
    "text": "Currently the obstacles are assumed to be represented by circles and their data is to be published to the vectors in the following rosparamName Description\n/obstacle/radius radius of obstacle in (m)\n/obstacle/vx global velocity in global x direction in (m/s)\n/obstacle/vy global velocity in global y direction in (m/s)\n/obstacle/x current global x (m) position of vehicle in (m)\n/obstacle/y current global y (m) position of vehicle in (m)"
},

{
    "location": "packages/nloptcontrol_planner/index.html#Vehicle-State-1",
    "page": "nloptcontrol_planner",
    "title": "Vehicle State",
    "category": "section",
    "text": "If an actual vehicle is used or an external model of the vehicle is used, /nloptcontrol_planner/flags/3DOF_plant should be set to false. And the following rosparam states (points) should be set:Name Description\n/state/x global x position (m)\n/state/y global y position (m)\n/state/psi global heading angle (rad)\n/state/sa steering angle at the tire (rad)\n/state/ux velocity in the x direction (vehicle frame) in (m/s)\n/state/ax acceleration in the x direction (vehicle frame) in (m/s^s)\n/state/r yaw rate about the z direction in (rad/s)"
},

{
    "location": "packages/nloptcontrol_planner/index.html#Outputs-1",
    "page": "nloptcontrol_planner",
    "title": "Outputs",
    "category": "section",
    "text": ""
},

{
    "location": "packages/nloptcontrol_planner/index.html#Trajectories-1",
    "page": "nloptcontrol_planner",
    "title": "Trajectories",
    "category": "section",
    "text": "The purpose of this node is to publish reference state trajectories (vectors) asName Description\n/nloptcontrol_planner/traj/t time (s)\n/nloptcontrol_planner/traj/x global x position trajectory (m)\n/nloptcontrol_planner/traj/y global y position trajectory (m)\n/nloptcontrol_planner/traj/psi global heading angle trajectory (rad)\n/nloptcontrol_planner/traj/sa steering angle trajectory at the tire (rad)\n/nloptcontrol_planner/traj/vx velocity trajectory in the x direction (vehicle frame) (m/s)"
},

{
    "location": "packages/obstacle_detector/index.html#",
    "page": "obstacle_detector",
    "title": "obstacle_detector",
    "category": "page",
    "text": ""
},

{
    "location": "packages/obstacle_detector/index.html#obstacle_detector-1",
    "page": "obstacle_detector",
    "title": "obstacle_detector",
    "category": "section",
    "text": "This obstacle detector is forked from obstacle_detector We made some modifications so that the package can detect and track obstacles from 3D PointCloud. Detected obstacles come in a form of circles. The working principles of the method are described in an article provided in the resources folder.This obstacle detection algorithms can predict the position (x,y), velocity (x,y), and size (assuming circular obstacles)."
},

{
    "location": "packages/obstacle_detector/index.html#Flags-and-Settings-1",
    "page": "obstacle_detector",
    "title": "Flags and Settings",
    "category": "section",
    "text": ""
},

{
    "location": "packages/obstacle_detector/index.html#Settings-1",
    "page": "obstacle_detector",
    "title": "Settings",
    "category": "section",
    "text": "Name Description\n/obstacle_detector/obstacle_extractor/active active/sleep mode\n/obstacle_detector/obstacle_extractor/use_scan use laser scan messages\n/obstacle_detector/obstacle_extractor/use_pcl use point cloud messages\n/obstacle_detector/obstacle_extractor/use_split_and_merge choose wether to use Iterative End Point Fit (false) or Split And Merge (true) algorithm to detect segments\n/obstacle_detector/obstacle_extractor/circles_from_visible detect circular obstacles only from fully visible (not occluded) segments\n/obstacle_detector/obstacle_extractor/discard_converted_segments do not publish segments, from which the circles were spawned\n/obstacle_detector/obstacle_extractor/min_group_points transform the coordinates of obstacles to a frame described with frame_id parameter\n/obstacle_detector/obstacle_extractor/transform_coordinates minimum number of points comprising a group to be further processed\n/obstacle_detector/obstacle_extractor/max_group_distance if the distance between two points is greater than this value, start a new group,\n/obstacle_detector/obstacle_extractor/distance_proportion enlarge the allowable distance between points proportionally to the range of point (use scan angle increment in radians)\n/obstacle_detector/obstacle_extractor/max_split_distance if a point in group lays further from a leading line than this value, split the group\n/obstacle_detector/obstacle_extractor/max_merge_separation if distance between obstacles is smaller than this value, consider merging them\n/obstacle_detector/obstacle_extractor/max_merge_spread merge two segments if all of their extreme points lay closer to the leading line than this value\n/obstacle_detector/obstacle_extractor/max_circle_radius if a circle would have greater radius than this value, skip it\n/obstacle_detector/obstacle_extractor/radius_enlargement artificially enlarge the circles radius by this value\n/obstacle_detector/obstacle_extractor/frame_id name of the coordinate frame used as origin for produced obstacles (used only if transform_coordinates flag is set to true)\n/obstacle_detector/obstacle_tracker/active active/sleep mode\n/obstacle_detector/obstacle_tracker/loop_rate the main loop rate in Hz\n/obstacle_detector/obstacle_tracker/tracking_duration the duration of obstacle tracking in the case of lack of incomming data\n/obstacle_detector/obstacle_tracker/min_correspondence_cost a threshold for correspondence test\n/obstacle_detector/obstacle_tracker/std_correspondence_dev (experimental) standard deviation of the position ellipse in the correspondence test\n/obstacle_detector/obstacle_tracker/process_variance variance of obstacles position and radius (parameter of Kalman Filter)\n/obstacle_detector/obstacle_tracker/process_rate_variance variance of rate of change of obstacles values (parameter of Kalman Filter)\n/obstacle_detector/obstacle_tracker/measurement_variance variance of measured obstacles values (parameter of Kalman Filter)\n/obstacle_detector/obstacle_tracker/frame_id name of the coordinate frame in which the obstacles are described\n/voxel_grid/filter_field_name the name of the point field to be used for filtering\n/voxel_grid/filter_limit_min The minimum limit of the filter interval\n/voxel_grid/filter_limit_max The maximum limit of the filter interval\n/voxel_grid/filter_limit_negative Inverts the meaning of the filter interval.\n/voxel_grid/leaf_size The extent of a leaf, respectively the voxel size of the result image or the size of the cells which shall accumulate points."
},

{
    "location": "packages/obstacle_detector/index.html#Flags-1",
    "page": "obstacle_detector",
    "title": "Flags",
    "category": "section",
    "text": "Name Description\n/system/obstacle_detector/flags/running indicates whether the obstacle detector is running\n/system/obstacle_detector/flags/initilized indicates whether the obstacle detector is initialized"
},

{
    "location": "packages/obstacle_detector/index.html#Input-1",
    "page": "obstacle_detector",
    "title": "Input",
    "category": "section",
    "text": "rostopic rosmsg\n/lidar_points PointCloud2"
},

{
    "location": "packages/obstacle_detector/index.html#Output-1",
    "page": "obstacle_detector",
    "title": "Output",
    "category": "section",
    "text": "Currently the obstacles are assumed to be represented by circles and their data is to be published to the vectors in the following rosparamName Description\n/obstacle/radius radius of obstacle in (m)\n/obstacle/vx global velocity in global x direction in (m/s)\n/obstacle/vy global velocity in global y direction in (m/s)\n/obstacle/x current global x (m) position of vehicle in (m)\n/obstacle/y current global y (m) position of vehicle in (m)"
},

{
    "location": "packages/vehicle_description/index.html#",
    "page": "vehicle_description",
    "title": "vehicle_description",
    "category": "page",
    "text": ""
},

{
    "location": "packages/vehicle_description/index.html#vehicle_description-1",
    "page": "vehicle_description",
    "title": "vehicle_description",
    "category": "section",
    "text": "Simulates a LiDAR in Gazebo"
},

{
    "location": "packages/vehicle_description/index.html#Flags-and-Settings-1",
    "page": "vehicle_description",
    "title": "Flags and Settings",
    "category": "section",
    "text": ""
},

{
    "location": "packages/vehicle_description/index.html#Input-1",
    "page": "vehicle_description",
    "title": "Input",
    "category": "section",
    "text": ""
},

{
    "location": "packages/vehicle_description/index.html#Output-1",
    "page": "vehicle_description",
    "title": "Output",
    "category": "section",
    "text": ""
},

{
    "location": "packages/ros_chrono/index.html#",
    "page": "ros_chrono",
    "title": "ros_chrono",
    "category": "page",
    "text": ""
},

{
    "location": "packages/ros_chrono/index.html#ros_chrono-1",
    "page": "ros_chrono",
    "title": "ros_chrono",
    "category": "section",
    "text": "A HMMWV vehicle model developed using Project Chrono is controlled using ROS parameters which transmit a desired path. The vehicle model is initialized with parameters from a config .yaml file, including an initial desired xy path. The vehicle can track to an updated path sent through the ROS parameter server. This is demonstrated by traj_gen_chrono.cpp updating the ROS parameters for the desired x and y coordinates after the vehicle begins tracking the initial desired path. The vehicle\'s states are published in a ROS msg and also saved as ROS parameters."
},

{
    "location": "packages/ros_chrono/index.html#Flags-and-Settings-1",
    "page": "ros_chrono",
    "title": "Flags and Settings",
    "category": "section",
    "text": ""
},

{
    "location": "packages/ros_chrono/index.html#Input-1",
    "page": "ros_chrono",
    "title": "Input",
    "category": "section",
    "text": ""
},

{
    "location": "packages/ros_chrono/index.html#Output-1",
    "page": "ros_chrono",
    "title": "Output",
    "category": "section",
    "text": ""
},

{
    "location": "packages/ros_chrono/index.html#Change-Vehicle-Initial-Conditions-1",
    "page": "ros_chrono",
    "title": "Change Vehicle Initial Conditions",
    "category": "section",
    "text": "To change initial trajectory edit the parameters in the hmmwv.yaml config file. To turn off the GUI (work in progress), change the value of system/chrono/flags/gui to false in test_chrono.yaml.$ sudo gedit ros/src/models/chrono/ros_chrono/config/hmmwv_params.yaml\n$ sudo gedit ros/src/system/config/vehicle/hmmwv.yaml\n$ sudo gedit ros/src/system/config/case1.yaml\n$ sudo gedit ros/src/system/system/test_chrono.yaml\n"
},

{
    "location": "packages/ros_chrono/index.html#Change-Values-of-Updated-Path-1",
    "page": "ros_chrono",
    "title": "Change Values of Updated Path",
    "category": "section",
    "text": "Change the values of x2, y2 in traj_gen_chrono.cpp and recompile using catkin_make. Change the system/planner parameter to default in global.yaml."
},

{
    "location": "packages/ros_chrono/index.html#Monitor-Vehicle-State-1",
    "page": "ros_chrono",
    "title": "Monitor Vehicle State",
    "category": "section",
    "text": "Open another terminal and type:$ rostopic echo vehicleinfo\nThis displays all states and inputs specified in the veh_status.msg file."
},

{
    "location": "packages/ros_chrono/index.html#Current-Differences-between-3DOF-Vehicle-model-and-HMMWV-model:-1",
    "page": "ros_chrono",
    "title": "Current Differences between 3DOF Vehicle model and HMMWV model:",
    "category": "section",
    "text": ""
},

{
    "location": "packages/ros_chrono/index.html#HMMWV-Model-1",
    "page": "ros_chrono",
    "title": "HMMWV Model",
    "category": "section",
    "text": "mass: 2,449.55696\nIzz: 3,570.2\nla: 1.871831 (Distance from COM to front axle)\nlb: 1.871831 (Distance from COM to rear axle)"
},

{
    "location": "packages/ros_chrono/index.html#DOF-Vehicle-Model-1",
    "page": "ros_chrono",
    "title": "3DOF Vehicle Model",
    "category": "section",
    "text": "mass: 2,688.7\nIzz: 4,110.1\nla: 1.5775\nlb: 1.7245  "
},

{
    "location": "packages/ros_chrono/index.html#Parameter-list-1",
    "page": "ros_chrono",
    "title": "Parameter list",
    "category": "section",
    "text": "/system/chrono/flags/gui (Switch to true or false)\n/case/X0/actual/ax (Initial x acceleration)\n/hmmwv_chrono/X0/theta (Initial pitch)\n/case/X0/actual/r (Initial r)\n/hmmwv_chrono/X0/theta (Initial roll)\n/case/X0/actual/sa (Initial steering angle)\n/case/X0/actual/ux (Initial x speed)\n/case/X0/actual/v (Initial velocity)\n/hmmwv_chrono/X0/v_des (Desired velocity)\n/case/X0/actual/x (Initial x)\n/case/X0/actual/yVal (Initial y)\n/case/X0/actual/psi (Initial yaw)\n/hmmwv_chrono/X0/z (Initial z)\n/vehicle/chrono/common/Izz (Moment of Inertia about z axis)\n/vehicle/chrono/common/la (Distance from COM to front axle)\n/vehicle/chrono/common/lb (Distance from COM to rear axle)\n/vehicle/chrono/common/mass (Vehicle mass)\n/vehicle/chrono/control/brk_in (Brake input)\n/vehicle/chrono/state/sa (Steering angle)\n/vehicle/chrono/control/str (Steering input)\n/vehicle/chrono/state/t (Time in chrono model)\n/vehicle/chrono/control/thr (Throttle input)\n/vehicle/chrono/state/ax (X acceleration)\n/vehicle/chrono/state/x (X position)\n/vehicle/chrono/state/ux (X speed)\n/vehicle/chrono/state/yVal (Y position)\n/vehicle/chrono/state/v (Y speed)\n/vehicle/chrono/state/psi (Yaw)\n/vehicle/chrono/state/r (Yaw rate)"
},

{
    "location": "packages/ros_chrono/index.html#Topic-list-1",
    "page": "ros_chrono",
    "title": "Topic list",
    "category": "section",
    "text": "/vehicleinfo (Vehicle states, inputs, and time)"
},

{
    "location": "packages/mavs_ros_planner/index.html#",
    "page": "mavs_ros_planner",
    "title": "mavs_ros_planner",
    "category": "page",
    "text": ""
},

{
    "location": "packages/mavs_ros_planner/index.html#mavs_ros_planner-1",
    "page": "mavs_ros_planner",
    "title": "mavs_ros_planner",
    "category": "section",
    "text": "This is a setup for Navigation Stack for the HMMWV model. This uses move_base ROS package and given a Start Pose and Goal pose, it publishes a trajectory into /cmd_vel.The setup uses navfn/NavfnROS as base_global_planner and base_local_planner/TrajectoryPlannerROS as base_local_planner"
},

{
    "location": "packages/mavs_ros_planner/index.html#Flags-and-Settings-1",
    "page": "mavs_ros_planner",
    "title": "Flags and Settings",
    "category": "section",
    "text": ""
},

{
    "location": "packages/mavs_ros_planner/index.html#Input-1",
    "page": "mavs_ros_planner",
    "title": "Input",
    "category": "section",
    "text": ""
},

{
    "location": "packages/mavs_ros_planner/index.html#Output-1",
    "page": "mavs_ros_planner",
    "title": "Output",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoA.html#",
    "page": "demoA",
    "title": "demoA",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoA.html#demoA-1",
    "page": "demoA",
    "title": "demoA",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoA.html#obstacle_detector-with-vehicle_description-1",
    "page": "demoA",
    "title": "obstacle_detector with vehicle_description",
    "category": "section",
    "text": "obstacle_detector with vehicle_description @tqshao can you please fix this?"
},

{
    "location": "demos/system/demoA.html#status-broken-1",
    "page": "demoA",
    "title": "status = broken",
    "category": "section",
    "text": "[ INFO] [1521730529.026696899, 0.483000000]: \"map\" passed to lookupTransform argument target_frame does not exist.\n[INFO] [1521730529.071362, 0.508000]: Spawn status: SpawnModel: Successfully spawned entity\n[Obstacle2/spawn_obstacles-6] process has finished cleanly\nlog file: /home/febbo/.ros/log/0717f39a-2de1-11e8-8052-104a7d04da99/Obstacle2-spawn_obstacles-6*.log\n[ INFO] [1521730529.252805167, 0.599000000]: \"map\" passed to lookupTransform argument target_frame does not exist.\n[Obstacle1/spawn_obstacles-5] process has finished cleanly\nlog file: /home/febbo/.ros/log/0717f39a-2de1-11e8-8052-104a7d04da99/Obstacle1-spawn_obstacles-5*.log\n[ INFO] [1521730529.431333339, 0.704000000]: \"map\" passed to lookupTransform argument target_frame does not exist."
},

{
    "location": "demos/system/demoA.html#To-Run-1",
    "page": "demoA",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoA.launch"
},

{
    "location": "demos/system/demoA.html#Expected-Output-1",
    "page": "demoA",
    "title": "Expected Output",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoB.html#",
    "page": "demoB",
    "title": "demoB",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoB.html#demoB-1",
    "page": "demoB",
    "title": "demoB",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoB.html#nloptcontrol_planner-with-vehicle_description-1",
    "page": "demoB",
    "title": "nloptcontrol_planner with vehicle_description",
    "category": "section",
    "text": "A demo that shows nloptcontrol_planner moving the vehicle_description vehicle within Gazebo based off of the solution to the OCP every 0.5 s."
},

{
    "location": "demos/system/demoB.html#status-needs-fix-1",
    "page": "demoB",
    "title": "status = needs fix",
    "category": "section",
    "text": "I do not see the path that is being planned displayed in RViz"
},

{
    "location": "demos/system/demoB.html#To-Run-1",
    "page": "demoB",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoB.launch"
},

{
    "location": "demos/system/demoB.html#Expected-Output-1",
    "page": "demoB",
    "title": "Expected Output",
    "category": "section",
    "text": "Gazebo should pop up and if you move the view so that you can see to the right (x,y)=(0,200), you will see the vehicle. All of the nodes are thin initialized and the nloptcontrol_planner node takes the longest, so for a few minutes the terminal screen will displaywaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...Eventually, the controller will be ready and the vehicle will start to move every time a new solution is generated. This is not closed loop, it just shows the connectivity of these nodes within a system."
},

{
    "location": "demos/system/demoC.html#",
    "page": "demoC",
    "title": "demoC",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoC.html#demoC-1",
    "page": "demoC",
    "title": "demoC",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoC.html#vehicle_description-and-ros_base_planner-1",
    "page": "demoC",
    "title": "vehicle_description and ros_base_planner",
    "category": "section",
    "text": "A demo that shows ros_base_planner calculating a path from the LiDAR data collected from the vehicle_description package."
},

{
    "location": "demos/system/demoC.html#status-needs-fix-1",
    "page": "demoC",
    "title": "status = needs fix",
    "category": "section",
    "text": "I do not see the path that is being planned displayed in RViz"
},

{
    "location": "demos/system/demoC.html#To-Run-1",
    "page": "demoC",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoC.launch"
},

{
    "location": "demos/system/demoC.html#Expected-Output-1",
    "page": "demoC",
    "title": "Expected Output",
    "category": "section",
    "text": "Gazebo should pop up and if you move the view so that you can see to the right (x,y)=(0,200), you will see the vehicle. All of the nodes are thin initialized and the nloptcontrol_planner node takes the longest, so for a few minutes the terminal screen will displaywaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...Eventually, the controller will be ready and the vehicle will start to move every time a new solution is generated. This is not closed loop, it just shows the connectivity of these nodes within a system."
},

{
    "location": "demos/system/demoD.html#",
    "page": "demoD",
    "title": "demoD",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoD.html#demoD-1",
    "page": "demoD",
    "title": "demoD",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoD.html#TODO-1",
    "page": "demoD",
    "title": "TODO",
    "category": "section",
    "text": "A demo that shows"
},

{
    "location": "demos/system/demoD.html#status-?-1",
    "page": "demoD",
    "title": "status = ?",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoD.html#To-Run-1",
    "page": "demoD",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoD.launch"
},

{
    "location": "demos/system/demoD.html#Expected-Output-1",
    "page": "demoD",
    "title": "Expected Output",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoE.html#",
    "page": "demoE",
    "title": "demoE",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoE.html#demoE-1",
    "page": "demoE",
    "title": "demoE",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoE.html#TODO-1",
    "page": "demoE",
    "title": "TODO",
    "category": "section",
    "text": "A demo that shows"
},

{
    "location": "demos/system/demoE.html#status-?-1",
    "page": "demoE",
    "title": "status = ?",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoE.html#To-Run-1",
    "page": "demoE",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoE.launch"
},

{
    "location": "demos/system/demoE.html#Expected-Output-1",
    "page": "demoE",
    "title": "Expected Output",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoF.html#",
    "page": "demoF",
    "title": "demoF",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoF.html#demoF-1",
    "page": "demoF",
    "title": "demoF",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoF.html#obstacle_detector-with-vehicle_description-and-nloptcontrol_planner-1",
    "page": "demoF",
    "title": "obstacle_detector with vehicle_description and nloptcontrol_planner",
    "category": "section",
    "text": "Same as system demoB.launch except, now obstacle_detector is used to detect the obstacle information and pass it to nloptcontrol_planner."
},

{
    "location": "demos/system/demoF.html#status-working-1",
    "page": "demoF",
    "title": "status = working",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoF.html#To-Run-1",
    "page": "demoF",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoF.launch"
},

{
    "location": "demos/system/demoF.html#Expected-Output-1",
    "page": "demoF",
    "title": "Expected Output",
    "category": "section",
    "text": "The only difference between this and system demoB.launch is that now an Rviz Display pops up and shows that the obstacle is being detected."
},

{
    "location": "demos/vehicle_description/demo.html#",
    "page": "vehicle_description",
    "title": "vehicle_description",
    "category": "page",
    "text": ""
},

{
    "location": "demos/vehicle_description/demo.html#vehicle_description-1",
    "page": "vehicle_description",
    "title": "vehicle_description",
    "category": "section",
    "text": "A stand-alone demo to show that the LiDAR model in the vehicle_description package is working and the position of the vehicle can be modified."
},

{
    "location": "demos/vehicle_description/demo.html#status-working-1",
    "page": "vehicle_description",
    "title": "status = working",
    "category": "section",
    "text": ""
},

{
    "location": "demos/vehicle_description/demo.html#To-Run-1",
    "page": "vehicle_description",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch vehicle_description demo.launch"
},

{
    "location": "demos/vehicle_description/demo.html#Expected-Output-1",
    "page": "vehicle_description",
    "title": "Expected Output",
    "category": "section",
    "text": "Gazebo should pop up and the vehicle starts to drive slowly through an obstacle field."
},

{
    "location": "demos/nloptcontrol_planner/demo.html#",
    "page": "nloptcontrol_planner",
    "title": "nloptcontrol_planner",
    "category": "page",
    "text": ""
},

{
    "location": "demos/nloptcontrol_planner/demo.html#nloptcontrol_planner-1",
    "page": "nloptcontrol_planner",
    "title": "nloptcontrol_planner",
    "category": "section",
    "text": "A stand-alone demo to show that the NLOptControl.jl is solving the OCP and connected to ROS."
},

{
    "location": "demos/nloptcontrol_planner/demo.html#status-working-1",
    "page": "nloptcontrol_planner",
    "title": "status = working",
    "category": "section",
    "text": ""
},

{
    "location": "demos/nloptcontrol_planner/demo.html#To-Run-1",
    "page": "nloptcontrol_planner",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch nloptcontrol_planner demo.launch"
},

{
    "location": "demos/nloptcontrol_planner/demo.html#Expected-Output-1",
    "page": "nloptcontrol_planner",
    "title": "Expected Output",
    "category": "section",
    "text": "After a few minutes the terminal should display:******************************************************************************\nThis program contains Ipopt, a library for large-scale nonlinear optimization.\n Ipopt is released as open source code under the Eclipse Public License (EPL).\n         For more information visit http://projects.coin-or.org/Ipopt\n******************************************************************************\n\nRunning model for the: 0 time\n[199.831, 8.44829, -0.304456, 0.225719, 1.62274, 0.0524729, 16.7114, -1.11083]\nnloptcontrol_planner has been initialized.\nRunning model for the: 1 time\n[198.976, 16.6123, -0.820356, 0.246638, 1.75329, 0.0330637, 16.1247, -1.22459]\nRunning model for the: 2 time\n[197.441, 24.3947, -0.579523, 0.0349721, 1.8273, -0.00889783, 15.6299, -0.77718]\nRunning model for the: 3 time\n[195.715, 31.9384, -0.0419052, -0.121407, 1.80044, -0.0317296, 15.3501, -0.361812]\nRunning model for the: 4 time\n[194.294, 39.4512, 0.237982, -0.16141, 1.72634, -0.0360165, 15.2689, 0.0185792]\nRunning model for the: 5 time\n[193.395, 47.0487, 0.302376, -0.149562, 1.64747, -0.0328082, 15.3683, 0.361693]\nRunning model for the: 6 time\n[193.034, 54.7811, 0.269118, -0.117383, 1.58031, -0.0253441, 15.6244, 0.646153]\nRunning model for the: 7 time\n[193.132, 62.6823, 0.204767, -0.0873723, 1.52942, -0.0189879, 16.0035, 0.854896]\nRunning model for the: 8 time\n[193.601, 70.7833, 0.149901, -0.0649146, 1.49162, -0.0140566, 16.4672, 0.985584]\nRunning model for the: 9 time\n[194.365, 79.1078, 0.10819, -0.0480866, 1.46356, -0.0102397, 16.9765, 1.03786]\nRunning model for the: 10 time\ngoal is in range\n[195.357, 87.6683, 0.0590599, -0.020078, 1.44614, -0.00305557, 17.4935, 1.02433]\nRunning model for the: 11 time\ngoal is in range\n[196.478, 96.4715, -0.0230271, 0.00710531, 1.44323, 0.00253812, 18.002, 1.00652]\nRunning model for the: 12 time\ngoal is in range\n[197.629, 105.525, -0.100926, 0.0220395, 1.45104, 0.00503927, 18.5022, 0.990981]\nRunning model for the: 13 time\ngoal is in range\n[198.736, 114.834, -0.150177, 0.027248, 1.46365, 0.00574419, 18.9955, 0.978923]\nRunning model for the: 14 time\ngoal is in range\n[199.758, 124.4, -0.175388, 0.0281426, 1.47758, 0.00578579, 19.4841, 0.974316]\nGoal Attained!\n\n[obstacle_avoidance-2] process has finished cleanly\nlog file: /home/febbo/.ros/log/f7108cac-2de8-11e8-8052-104a7d04da99/obstacle_avoidance-2*.logThis indicates a successful test."
},

{
    "location": "demos/nloptcontrol_planner/demo.html#Notes-1",
    "page": "nloptcontrol_planner",
    "title": "Notes",
    "category": "section",
    "text": "A large optimization problem needs to be initialized\ncaching the functions upon start-up takes a few minutes"
},

{
    "location": "demos/ros_chrono/demo.html#",
    "page": "ros_chrono",
    "title": "ros_chrono",
    "category": "page",
    "text": ""
},

{
    "location": "demos/ros_chrono/demo.html#ros_chrono-1",
    "page": "ros_chrono",
    "title": "ros_chrono",
    "category": "section",
    "text": "A vehicle model in Chrono that can be used through ROS."
},

{
    "location": "demos/ros_chrono/demo.html#status-?-1",
    "page": "ros_chrono",
    "title": "status = ?",
    "category": "section",
    "text": ""
},

{
    "location": "demos/ros_chrono/demo.html#To-run-1",
    "page": "ros_chrono",
    "title": "To run",
    "category": "section",
    "text": "$ cd $HOME/MAVs/ros\n$ roslaunch ros_chrono path_follower.launch\n"
},

{
    "location": "demos/ros_chrono/demo.html#Expected-Output-1",
    "page": "ros_chrono",
    "title": "Expected Output",
    "category": "section",
    "text": ""
},

{
    "location": "demos/mavs_ros_planner/demo.html#",
    "page": "mavs_ros_planner",
    "title": "mavs_ros_planner",
    "category": "page",
    "text": ""
},

{
    "location": "demos/mavs_ros_planner/demo.html#mavs_ros_planner-1",
    "page": "mavs_ros_planner",
    "title": "mavs_ros_planner",
    "category": "section",
    "text": ""
},

{
    "location": "demos/mavs_ros_planner/demo.html#status-?-1",
    "page": "mavs_ros_planner",
    "title": "status = ?",
    "category": "section",
    "text": ""
},

{
    "location": "demos/mavs_ros_planner/demo.html#To-run-1",
    "page": "mavs_ros_planner",
    "title": "To run",
    "category": "section",
    "text": "Start Gazeboroslaunch mavs_gazebo demo.launchRun RViz and load the config file from MAVs/ros/src/system/config/planner/ros_base_planner/default.rviz\nSelect a 2D Pose Estimate and 2D Nav Goal in RViz. Make sure that Goal is within Global Cost map\nBase planner would create a trajectory which would be visible in RViz"
},

{
    "location": "demos/mavs_ros_planner/demo.html#Expected-Output-1",
    "page": "mavs_ros_planner",
    "title": "Expected Output",
    "category": "section",
    "text": ""
},

{
    "location": "issues/index.html#",
    "page": "Potential Issues",
    "title": "Potential Issues",
    "category": "page",
    "text": ""
},

{
    "location": "issues/index.html#Potential-Issues-1",
    "page": "Potential Issues",
    "title": "Potential Issues",
    "category": "section",
    "text": ""
},

{
    "location": "issues/index.html#julia-binaries-are-located-in-a-different-folder-than-/usr/bin-1",
    "page": "Potential Issues",
    "title": "julia binaries are located in a different folder than /usr/bin",
    "category": "section",
    "text": "In such a case you may try to hint to the binary as:#!/opt/julia/bin/env juliaThen after making your script to make it executable with:$ chmod a+x main.jlYou kick off a roscore and run:$ rosrun bot_description main.jlWith the resulting error:/opt/ros/kinetic/bin/rosrun: /home/febbo/catkin_ws/src/bot_description/main.jl: /opt/julia/bin: bad interpreter: Permission denied\n/opt/ros/kinetic/bin/rosrun: line 109: /home/febbo/catkin_ws/src/bot_description/main.jl: SuccessThen after making sure that the binary is in the correct location:$ /opt/julia/bin/julia -e \'println(\"Hello world\")\'\nHello world"
},

{
    "location": "issues/index.html#A-fix-is-to-make-a-link-to-the-binary-in-the-usr/bin-directory-1",
    "page": "Potential Issues",
    "title": "A fix is to make a link to the binary in the usr/bin directory",
    "category": "section",
    "text": "(Image: link)"
},

]}
