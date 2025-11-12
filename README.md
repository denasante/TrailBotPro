# 41068 Ignition Bringup

Bringup for *41068 Robotics Studio I*. Launches a Husky robot in a custom simulation world with trees and grass. We use **ROS2 Humble** and **Ignition Gazebo Fortress**.

Added 05/09/2025: Also included a drone robot ("parrot"). Scroll down for instructions specifically for the drone.

Worlds are build from [Gazebo Fuel](https://app.gazebosim.org/fuel/models).

## Installation

First install some dependencies:

* If you haven't already, install ROS2 Humble. Follow the instructions here: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
* Install Gazebo
  ```bash
  sudo apt-get update && sudo apt-get install wget
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update && sudo apt-get install ignition-fortress
  ```
* Install development tools and robot localisation
  ```bash
  sudo apt install ros-dev-tools ros-humble-robot-localization
  sudo apt install ros-humble-ros-ign ros-humble-ros-ign-interfaces
  sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes
  ```
* Make sure that your installation is up to date. This is particularly important if you installed ROS a long time ago, such as in another subject. If you get errors here, make sure to resolve these before continuing.
  ```bash
  sudo apt upgrade
  sudo apt update
  ```  

Now install this package:
* Create a new colcon workspace
  ```bash
  mkdir -p 41068_ws/src
  ```
* Copy this package to the `src` directory in this workspace
* Build package. If you get an error suggesting a missing dependency, make sure you have followed all of the above installation instructions correctly.
  ```bash
  source /opt/ros/humble/setup.bash
  cd 41068_ws
  colcon build --symlink-install
  ```
* Source workspace (if you add this to your ~/.bashrc, then you don't need to do this each time)
  ```bash
  source ~/41068_ws/install/setup.bash
  ```
* Launch basic trees world. It might take a little while to load the first time you run it since it is downloading world model resources. If it crashes the first time, try running it again.
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py
  ```
* As above with SLAM and autonomous navigation
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true
  ```
* Change world with `world` argument. Must be the name of a `.sdf` file in `worlds`, but without file extension. Note this might also take a while the first time you run it since it is downloading extra model resources.
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=large_demo
  ```
* And similarly, the larger world, and with SLAM and navigation:
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true world:=large_demo
  ```
* When launching with rviz, you can send a waypoint to the robot by clicking the "2D Goal pose" and then a location in the map. The robot is navigating using the nav2 package. If it gets stuck, you can try the buttons in the Navigation 2 panel in the top right of RVIZ.

* You can also drive the robot using keyboard teleoperation by running the following in a separate terminal, then use the keys listed in the instructions to move the robot:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

## Drone (added 05/09/2025)

By popular request, I've added a simple drone to the package, which requires a few modifications to get running:

* In the world file (large_demo.sdf or simple_trees.sdf, etc.), set the gravity to 0, so the drone doesn't fall out of the sky. I'll let you work out how to do that.

* I've created a separate launch file for the drone, which is almost the same, except spawns a "parrot" drone instead of the husky:
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py slam:=true nav2:=true rviz:=true world:=large_demo
  ```

* At the moment, the drone will just fly at a fixed altitude. You can change the altitude in 41068_ignition_drone.launch.py. Find the `robot_spawner` node, and change change the `z` parameter, which is height above the ground where it is spawned.

* The drone's camera is setup to tilt 45 degrees downwards (looking slightly down towards the ground). You can change this in `urdf_drone/parrot.urdf.xacro`, find the `camera_joint` and change the "pitch" (you'll need to workout how to do this exactly).

* Since the drone has a very hard time navigating through the leaves of the trees, I've disabled the collisions of the drones. So at the moment, it is allowed to fly through objects. You can enable collisions by uncommenting the `collision` field in `urdf_drone/parrot.urdf.xacro`. Note that the default navigation stack doesn't work well in this situation, so you will need to further develop the collision avoidance planners for this new challenge!

* At the moment, you can only run the drone or the husky, not both. If you wish to spawn both at the same time, then you'll need to figure out how :)



## Standalone ROS 2 Nodes

Again, make sure the workspace is built and `source install/setup.bash` has been run so `ros2 run` can find the executables.

### Autonomy & perception nodes (`ros2 run 41068_ignition_bringup <executable>`)

- **`dronecontroller`** (`dronecontroller.cpp`): Lawn-mower coverage controller for the Parrot with LiDAR-based obstacle avoidance (`/odometry`, `/scan` → `cmd_vel`).  
  Launch: `ros2 run 41068_ignition_bringup dronecontroller --ros-args -p min_x:=-12.0 -p max_x:=12.0 -p scan_topic:=/parrot/scan`

- **`rover_controller`** (`rover_controller.cpp`): Frontier-based waypoint generator for Husky (`/map` → `/waypoint`).  
  Launch: `ros2 run 41068_ignition_bringup rover_controller --ros-args -p map_frame:=husky/map -p goal_offset_m:=0.5`

- **`mission_interface`** (`mission_interface.cpp`): Nav2 NavigateToPose action client that streams `/waypoint` topics or CSV paths to Nav2.  
  Launch: `ros2 run 41068_ignition_bringup mission_interface --ros-args -p path_csv:=/tmp/mission.csv -p loop:=true`

- **`drone_object_detector_bamboo.py`**: HSV + depth based bamboo-cluster detector publishing `/trail_guardian/detections`, `/trail_guardian/anomaly_location`, `/trail_guardian/detection_markers`.  
  Launch: `ros2 run 41068_ignition_bringup drone_object_detector_bamboo.py --ros-args -p visualization:=true -p detection_rate:=2.0`

- **`tree_detector.py`**: Oak/pine detector that fuses color, shape, and depth cues and republishes markers + `/trail_guardian/tree_location`.  
  Launch: `ros2 run 41068_ignition_bringup tree_detector.py --ros-args -p detection_rate:=0.5`

- **`square_pattern_flight.py`**: Publishes `/parrot/goal_pose` waypoints to fly a calibration square for perception testing.  
  Launch: `ros2 run 41068_ignition_bringup square_pattern_flight.py --ros-args -p square_size:=12.0 -p flight_height:=3.5`

- **`frontier_explorer.py`** (`scripts/frontier_explorer.py`): Consumes an occupancy grid (`/parrot/map`) and publishes frontier goals to `/parrot/goal_pose`, plus RViz markers.  
  Launch: `ros2 run 41068_ignition_bringup frontier_explorer.py --ros-args -p exploration_radius:=20.0`

- **`object_landmark_mapper.py`** (`scripts/object_landmark_mapper.py`): RGB-D landmark mapper that deduplicates detections and exports GeoJSON/CSV plus `/unique_objects_viz`.  
  Launch: `ros2 run 41068_ignition_bringup object_landmark_mapper.py --ros-args -p output_dir:=/tmp/landmarks -p dedup_radius:=0.6`

- **`drone_goal_publisher.py`** (`scripts/drone_goal_publisher.py`): Convenience Nav2 goal sender (CLI or command-line coordinates) for any namespace.  
  Launch: `ros2 run 41068_ignition_bringup drone_goal_publisher.py --ros-args -p namespace:=parrot`

### Teleop, bridging, and infrastructure

- **`drone_keyboard_teleop.py`**: Raw terminal teleop for the Parrot (`/parrot/cmd_vel`). Run in a dedicated terminal with focus on the window.  
  Launch: `ros2 run 41068_ignition_bringup drone_keyboard_teleop.py`

- **`frame_id_relay.py`** (`scripts/frame_id_relay.py`): Generic relay that rewrites `header.frame_id` / `child_frame_id` and can also broadcast TF for odometry streams.  
  Launch: `ros2 run 41068_ignition_bringup frame_id_relay.py --ros-args -p input_topic:=/husky/odometry/raw -p output_topic:=/husky/odometry -p message_type:=nav_msgs/msg/Odometry -p frame_id:=husky/odom -p child_frame_id:=husky/base_link -p broadcast_tf:=true`

- **`static_camera_info_pub`** (`static_camera_info_pub.cpp`): Publishes `sensor_msgs/CameraInfo` matching a YAML file whenever `/camera/image` arrives (useful for AprilTag).  
  Launch: `ros2 run 41068_ignition_bringup static_camera_info_pub --ros-args -p camera_info_yaml:=$(ros2 pkg prefix 41068_ignition_bringup)/share/41068_ignition_bringup/config/camera_info.yaml`

### Developer-only scripts (run from the package root)

- **`autonomous_scout.py`**: High-level finite-state autonomous scout for the Parrot (coverage path + POI reporting). Run with `python3 autonomous_scout.py` after sourcing the workspace; it talks to `/parrot/odometry`, `/parrot/cmd_vel`, and `/trail_guardian/*` topics.

- **`control_ui.py`**: Tkinter control surface for Husky teleop (`/husky/cmd_vel`) plus status channel. Either run `python3 control_ui.py` or use the packaged launch file (`ros2 launch 41068_ignition_bringup ui_launch.py`) once you have a GUI session.

- **`drone_object_detector copy.py`**: Legacy hazard detector variant that looks for erosion/obstacle colors instead of bamboo. Run manually via `python3 "drone_object_detector copy.py"` if you still need that pipeline.


## Errors

If you are getting errors, first check you are following the instructions correctly. Otherwise, read the error messages carefully and google it or discuss with your team or the teaching staff. Here's two errors I came across and fixes.

### Jump back in time

If you continuously get an error like:

```bash
Detected jump back in time. Clearing TF buffer
```

and you probably see things flashing in rviz, then this is probably due to the simulation clock time being reset constantly. This is likely caused by multiple gazebo instances running, perhaps a crashed gazebo in the background that didn't close properly. 

To fix this, I suggest restarting the computer. 

### Ogre Exception

If you get an error like:

```bash
[Ogre2RenderEngine.cc:989]  Unable to create the rendering window: OGRE EXCEPTION(3:RenderingAPIException): currentGLContext was specified with no current GL context in GLXWindow::create at /build/ogre-next-UFfg83/ogre-next-2.2.5+dfsg3/RenderSystems/GL3Plus/src/windowing/GLX/OgreGLXWindow.cpp (line 163)
```

I found [this thread](https://robotics.stackexchange.com/questions/111547/gazebo-crashes-immediately-segmentation-fault-address-not-mapped-to-object-0) which suggests to set a bash variable before launching Gazebo:

```bash
export QT_QPA_PLATFORM=xcb
```
## Launch Files At-a-Glance

All `ros2 launch` examples below assume you have already built the workspace with `colcon build --symlink-install` and sourced `install/setup.bash`.

| Launch file | What it starts | Key arguments | Command |
| --- | --- | --- | --- |
| `41068_ignition.launch.py` | Husky in Ignition Gazebo (robot_state_publisher, robot_localization EKF, ROS⇔Ign bridge, optional SLAM/Nav2 + RViz) | `world`, `nav2`, `rviz`, `use_sim_time` | `ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=simple_trees nav2:=true rviz:=true` |
| `41068_ignition_drone.launch.py` | Parrot drone only: Gazebo spawn, EKF, ROS⇔Ign bridges, static camera info pub, AprilTag node, `dronecontroller`, optional Nav2/RViz | `world`, `nav2`, `rviz`, `use_sim_time` | `ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py world:=forest_trail nav2:=true` |
| `trail_guardian_autonomous_explorer.launch.py` | Stage-by-stage bring-up tuned for the autonomous explorer mission (Gazebo, Parrot spawn, EKF, SLAM Toolbox, direct Nav2 launch, RViz) | `world`, `enable_slam`, `enable_nav2`, `rviz` | `ros2 launch 41068_ignition_bringup trail_guardian_autonomous_explorer.launch.py world:=large_demo enable_nav2:=true` |
| `trail_guardian_dual_simplified.py` | Simplified dual-robot launch (Husky + Parrot) with namespaced robot_state_publishers, EKFs, Ign bridges, and frame relays | `world`, `start_husky`, `start_drone`, `nav2`, `rviz`, `use_sim_time` | `ros2 launch 41068_ignition_bringup trail_guardian_dual_simplified.py start_husky:=true start_drone:=true nav2:=false` |
| `dual_sim.launch.py` | Feature-rich dual simulation with optional sensor stripping, headless Ignition mode, and Nav2 toggles | `world_file`, `start_rover`, `start_drone`, `rviz`, `nav2`, `strip_cameras_on`, `headless` | `ros2 launch 41068_ignition_bringup dual_sim.launch.py start_rover:=true start_drone:=false strip_cameras_on:=parrot` |
| `41068_navigation.launch.py` | SLAM Toolbox + Nav2 stack for Husky (also included from the other launchers) | `use_sim_time` | `ros2 launch 41068_ignition_bringup 41068_navigation.launch.py use_sim_time:=true` |
| `parrot_nav2.launch.py` | Full Nav2 stack for the Parrot namespace using YAML parameters (controller, planner, behaviors, BT navigator, waypoint follower, velocity smoother) | `namespace`, `params_file`, `autostart`, `use_sim_time` | `ros2 launch 41068_ignition_bringup parrot_nav2.launch.py namespace:=parrot use_sim_time:=true` |
| `parrot_nav2_direct_launch.py` | Hard-coded Nav2 configuration (no YAML) used by the autonomous explorer launch | `namespace`, `use_sim_time` | `ros2 launch 41068_ignition_bringup parrot_nav2_direct_launch.py namespace:=parrot` |
| `"Test husky only launch.py"` | Minimal Husky-only Gazebo bring-up for debugging (rsp, EKF, Ign bridge) | `world`, `use_sim_time` | `ros2 launch 41068_ignition_bringup "Test husky only launch.py" world:=simple_trees` |
| `ui_launch.py` | Starts the Tk-based `control_ui` node so you can drive Husky via GUI buttons/WASD | *(none)* | `ros2 launch 41068_ignition_bringup ui_launch.py` |
