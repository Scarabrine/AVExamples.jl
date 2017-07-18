# Controlling a Gazebo Model

Note: This is a working progress!

This is an example that demonstrates how to control a simple ``Gazebo`` model using ``RobotOS.jl`` through ``ROS``.


For now, this tutorial assumes that the user has a basic understanding of ``Gazebo`` and ``ROS`` and is running the ``ros-kinetic-desktop-full`` version of ``ROS`` on Ubuntu 16.04.


## ``bot.urdf.xacro`` file

    <?xml version="1.0"?>
    <robot  name="fighter" xmlns:xacro="http://www.ros.org/wiki/xacro">

      <xacro:property name="width" value="0.2" />
      <xacro:property name="length" value="0.6" />
      <xacro:property name="height" value="0.6" />

      <xacro:property name="x" value="0" />
      <xacro:property name="y" value="0" />
      <xacro:property name="z" value="0.6" />

      <xacro:property name="roll" value="0" />
      <xacro:property name="pitch" value="0" />
      <xacro:property name="yaw" value="0" />

      <!-- Base Footprint -->
      <link name="base_footprint" />

      <!-- Base Link -->
      <joint name="footprint" type="fixed" >
        <parent link="base_footprint" />
        <child link="base_link" />
        <origin xyz="${x} ${y} ${z}" rpy="${roll} ${pitch} ${yaw}" />
      </joint>

      <link name="base_link" >
        <visual>
          <geometry>
            <box size="${length} ${width} ${height}" />
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box size="${length} ${width} ${height}" />
          </geometry>
        </collision>
        <inertial>
          <origin xyz="${x} ${y} ${z}"/>
          <mass value="10"/>
          <inertia ixx="3.0" ixy="0.0" ixz="0.0"
                   iyy="3.0" iyz="0.0"
                   izz="3.0" />
        </inertial>
      </link>


    </robot>

## ``example.launch`` file

    <?xml version="1.0" ?>
    <launch>
      <arg name="paused" default="false"/>
      <arg name="use_sim_time" default="true"/>
      <arg name="gui" default="true"/>
      <arg name="headless" default="false"/>
      <arg name="debug" default="false"/>
      <arg name="verbose" default="false"/>
      <arg name="world_name" default="worlds/empty.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->

      <!-- Start gazebo and load the world -->
      <include file="$(find gazebo_ros)/launch/empty_world.launch" >
        <arg name="paused" value="$(arg paused)"/>
        <arg name="use_sim_time" value="$(arg use_sim_time)"/>
        <arg name="gui" value="$(arg gui)"/>
        <arg name="headless" value="$(arg headless)"/>
        <arg name="debug" value="$(arg debug)"/>
        <arg name="verbose" value="$(arg verbose)"/>
        <arg name="world_name" value="$(arg world_name)"/>
      </include>

      <!-- Spawn the Base -->
      <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find bot_description)/urdf/bot.urdf.xacro'" />
      <node pkg="gazebo_ros" type="spawn_model" name="spawn_model" args="-param /robot_description -urdf -model base_example"/>
      <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
        <param name="publish_frequency" type="double" value="30.0" />
      </node>

    </launch>


## package.xml

    <?xml version="1.0"?>
    <package format="2">
      <name>bot_description</name>
      <version>0.0.0</version>
      <description>The bot_description package</description>
      <license>MIT</license>
      <author email="febboh@umich.edu">Huckleberry Febbo</author>
      <maintainer email="febboh@umich.edu">Huckleberry Febbo</maintainer>
      <buildtool_depend>catkin</buildtool_depend>
      <depend>gazebo_ros</depend>
      <depend>gazebo_msgs</depend>
      <depend>geometry_msgs</depend>
      <depend>std_srvs</depend>
    </package>

## CMakeLists.txt

    cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
    find_package(gazebo REQUIRED)
    include_directories(${GAZEBO_INCLUDE_DIRS})
    link_directories(${GAZEBO_LIBRARY_DIRS})
    list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


## Run the example

  roslaunch bot_description example.launch


## julia code

    using RobotOS
    @rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3
    @rosimport std_srvs.srv: Empty, SetBool
    @rosimport nav_msgs.srv.GetPlan

    @rosimport gazebo_msgs.msg: ModelState
    @rosimport gazebo_msgs.srv: SetModelState

    rostypegen()
    using geometry_msgs.msg
    using std_srvs.srv
    using nav_msgs.srv.GetPlan
    using gazebo_msgs.msg
    using gazebo_msgs.srv

    # Set up services
    init_node("rosjl_ex")
    const set_state = ServiceProxy("/gazebo/set_model_state", SetModelState)
    println("Waiting for 'gazebo/set_model_state' service...")
    wait_for_service("gazebo/set_model_state")

    # Define position to move robot
    vehPose=Pose()
    vehPose.position.x = 10*rand()
    ##TODO get the current position of the robot, currently just setting all of the states except x to zero

    # Define the state of the Gazebo model
    state = ModelState()
    state.model_name = "base_example"
    state.pose = vehPose

    println("Calling 'gazebo/set_model_state' service...")
    set_state(state)


Currently the above example is run directly in the ``julia`` terminal and there is an error:

    julia> set_state(state)
    ERROR: MethodError: no method matching (::RobotOS.ServiceProxy{gazebo_msgs.srv.SetModelState})(::gazebo_msgs.msg.ModelState)
