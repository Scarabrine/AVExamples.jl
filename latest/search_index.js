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
    "location": "index.html#Exported-Functions-1",
    "page": "Home",
    "title": "Exported Functions",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoA.html#",
    "page": "-",
    "title": "-",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoA.html#obstacle_detector-with-vehicle_description-1",
    "page": "-",
    "title": "obstacle_detector with vehicle_description",
    "category": "section",
    "text": "obstacle_detector with vehicle_description @tqshao can you please fix this?"
},

{
    "location": "demos/system/demoA.html#status-broken-1",
    "page": "-",
    "title": "status = broken",
    "category": "section",
    "text": "[ INFO] [1521730529.026696899, 0.483000000]: \"map\" passed to lookupTransform argument target_frame does not exist.\n[INFO] [1521730529.071362, 0.508000]: Spawn status: SpawnModel: Successfully spawned entity\n[Obstacle2/spawn_obstacles-6] process has finished cleanly\nlog file: /home/febbo/.ros/log/0717f39a-2de1-11e8-8052-104a7d04da99/Obstacle2-spawn_obstacles-6*.log\n[ INFO] [1521730529.252805167, 0.599000000]: \"map\" passed to lookupTransform argument target_frame does not exist.\n[Obstacle1/spawn_obstacles-5] process has finished cleanly\nlog file: /home/febbo/.ros/log/0717f39a-2de1-11e8-8052-104a7d04da99/Obstacle1-spawn_obstacles-5*.log\n[ INFO] [1521730529.431333339, 0.704000000]: \"map\" passed to lookupTransform argument target_frame does not exist."
},

{
    "location": "demos/system/demoA.html#To-Run-1",
    "page": "-",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoA.launch"
},

{
    "location": "demos/system/demoB.html#",
    "page": "nloptcontrol_planner with vehicle_description",
    "title": "nloptcontrol_planner with vehicle_description",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoB.html#nloptcontrol_planner-with-vehicle_description-1",
    "page": "nloptcontrol_planner with vehicle_description",
    "title": "nloptcontrol_planner with vehicle_description",
    "category": "section",
    "text": "A demo that shows NLOptControl.jl moving the vehicle_description vehicle within Gazebo based off of the solution to the OCP every 05 s."
},

{
    "location": "demos/system/demoB.html#status-needs-fix-1",
    "page": "nloptcontrol_planner with vehicle_description",
    "title": "status = needs fix",
    "category": "section",
    "text": "I do not see the path that is being planned displayed in RViz"
},

{
    "location": "demos/system/demoB.html#To-Run-1",
    "page": "nloptcontrol_planner with vehicle_description",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoB.launch"
},

{
    "location": "demos/system/demoB.html#Expected-Output-1",
    "page": "nloptcontrol_planner with vehicle_description",
    "title": "Expected Output",
    "category": "section",
    "text": "Gazebo should pop up and if you move the view so that you can see to the right (xy)=(0200), you will see the vehicle. All of the nodes are thin initialized and the nloptcontrol_planner node takes the longest, so for a few minutes the terminal screen will displaywaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...Eventually, the controller will be ready and the vehicle will start to move every time a new solution is generated. This is not closed loop, it just shows the connectivity of these nodes within a system."
},

{
    "location": "demos/system/demoC.html#",
    "page": "vehicle_description and ros_base_planner",
    "title": "vehicle_description and ros_base_planner",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoC.html#vehicle_description-and-ros_base_planner-1",
    "page": "vehicle_description and ros_base_planner",
    "title": "vehicle_description and ros_base_planner",
    "category": "section",
    "text": "A demo that shows ros_base_planner calculating a path from the LiDAR data collected from the vehicle_description package."
},

{
    "location": "demos/system/demoC.html#status-needs-fix-1",
    "page": "vehicle_description and ros_base_planner",
    "title": "status = needs fix",
    "category": "section",
    "text": "I do not see the path that is being planned displayed in RViz"
},

{
    "location": "demos/system/demoC.html#To-Run-1",
    "page": "vehicle_description and ros_base_planner",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoC.launch"
},

{
    "location": "demos/system/demoC.html#Expected-Output-1",
    "page": "vehicle_description and ros_base_planner",
    "title": "Expected Output",
    "category": "section",
    "text": "Gazebo should pop up and if you move the view so that you can see to the right (xy)=(0200), you will see the vehicle. All of the nodes are thin initialized and the nloptcontrol_planner node takes the longest, so for a few minutes the terminal screen will displaywaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...Eventually, the controller will be ready and the vehicle will start to move every time a new solution is generated. This is not closed loop, it just shows the connectivity of these nodes within a system."
},

{
    "location": "demos/system/demoD.html#",
    "page": "-",
    "title": "-",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoE.html#",
    "page": "-",
    "title": "-",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoF.html#",
    "page": "obstacle_detector with vehicle_description and nloptcontrol_planner",
    "title": "obstacle_detector with vehicle_description and nloptcontrol_planner",
    "category": "page",
    "text": ""
},

{
    "location": "demos/system/demoF.html#obstacle_detector-with-vehicle_description-and-nloptcontrol_planner-1",
    "page": "obstacle_detector with vehicle_description and nloptcontrol_planner",
    "title": "obstacle_detector with vehicle_description and nloptcontrol_planner",
    "category": "section",
    "text": "Same as system demoBlaunch except, now obstacle_detector is used to detect the obstacle information and pass it to nloptcontrol_planner."
},

{
    "location": "demos/system/demoF.html#status-working-1",
    "page": "obstacle_detector with vehicle_description and nloptcontrol_planner",
    "title": "status = working",
    "category": "section",
    "text": ""
},

{
    "location": "demos/system/demoF.html#Expected-Output-1",
    "page": "obstacle_detector with vehicle_description and nloptcontrol_planner",
    "title": "Expected Output",
    "category": "section",
    "text": "The only difference between this and system demoBlaunch is that now an Rviz Display pops up and shows that the obstacle is being detected."
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
    "text": "Gazebo should pop up and the vehicle starts to drive slowly through an obstacle field. "
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
    "text": "After a few minutes the terminal should display:******************************************************************************\nThis program contains Ipopt, a library for large-scale nonlinear optimization.\n Ipopt is released as open source code under the Eclipse Public License (EPL).\n         For more information visit http://projects.coin-or.org/Ipopt\n******************************************************************************\n\nRunning model for the: 0 time\n[199.831, 8.44829, -0.304456, 0.225719, 1.62274, 0.0524729, 16.7114, -1.11083]\nnloptcontrol_planner has been initialized.\nRunning model for the: 1 time\n[198.976, 16.6123, -0.820356, 0.246638, 1.75329, 0.0330637, 16.1247, -1.22459]\nRunning model for the: 2 time\n[197.441, 24.3947, -0.579523, 0.0349721, 1.8273, -0.00889783, 15.6299, -0.77718]\nRunning model for the: 3 time\n[195.715, 31.9384, -0.0419052, -0.121407, 1.80044, -0.0317296, 15.3501, -0.361812]\nRunning model for the: 4 time\n[194.294, 39.4512, 0.237982, -0.16141, 1.72634, -0.0360165, 15.2689, 0.0185792]\nRunning model for the: 5 time\n[193.395, 47.0487, 0.302376, -0.149562, 1.64747, -0.0328082, 15.3683, 0.361693]\nRunning model for the: 6 time\n[193.034, 54.7811, 0.269118, -0.117383, 1.58031, -0.0253441, 15.6244, 0.646153]\nRunning model for the: 7 time\n[193.132, 62.6823, 0.204767, -0.0873723, 1.52942, -0.0189879, 16.0035, 0.854896]\nRunning model for the: 8 time\n[193.601, 70.7833, 0.149901, -0.0649146, 1.49162, -0.0140566, 16.4672, 0.985584]\nRunning model for the: 9 time\n[194.365, 79.1078, 0.10819, -0.0480866, 1.46356, -0.0102397, 16.9765, 1.03786]\nRunning model for the: 10 time\ngoal is in range\n[195.357, 87.6683, 0.0590599, -0.020078, 1.44614, -0.00305557, 17.4935, 1.02433]\nRunning model for the: 11 time\ngoal is in range\n[196.478, 96.4715, -0.0230271, 0.00710531, 1.44323, 0.00253812, 18.002, 1.00652]\nRunning model for the: 12 time\ngoal is in range\n[197.629, 105.525, -0.100926, 0.0220395, 1.45104, 0.00503927, 18.5022, 0.990981]\nRunning model for the: 13 time\ngoal is in range\n[198.736, 114.834, -0.150177, 0.027248, 1.46365, 0.00574419, 18.9955, 0.978923]\nRunning model for the: 14 time\ngoal is in range\n[199.758, 124.4, -0.175388, 0.0281426, 1.47758, 0.00578579, 19.4841, 0.974316]\nGoal Attained!\n\n[obstacle_avoidance-2] process has finished cleanly\nlog file: /home/febbo/.ros/log/f7108cac-2de8-11e8-8052-104a7d04da99/obstacle_avoidance-2*.log"
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
    "location": "mavs/index.html#",
    "page": "MAVs.jl",
    "title": "MAVs.jl",
    "category": "page",
    "text": ""
},

{
    "location": "mavs/index.html#MAVs.jl-1",
    "page": "MAVs.jl",
    "title": "MAVs.jl",
    "category": "section",
    "text": "MAVs.jl is a julia package that solves an autonomous vehicle motion planning problem using NLOptControl.jl.There are several different modules."
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
