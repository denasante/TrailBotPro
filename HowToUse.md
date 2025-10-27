# Always do these two sources in any new terminal:
--
source /opt/ros/humble/setup.bash
cd ~/git/TrailBotPro
colcon build --symlink-install
source install/setup.bash
--

#  Step 1 
# Launch the simulator (Gazebo + bridges + RViz)

# Terminal 1

# Rover-only world
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py rviz:=True

OR

# Drone-only world
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py rviz:=True


# Step 2 
# Terminal 2

# Just Rover
source /opt/ros/humble/setup.bash
source ~/git/TrailBotPro/install/setup.bash

ros2 run 41068_ignition_bringup rover_controller \
  --ros-args -p odom_topic:=/odometry -p cmd_vel_topic:=/cmd_vel


# Just Drone
ros2 run 41068_ignition_bringup dronecontroller --ros-args \
  -p odom_topic:=/parrot/odometry \
  -p cmd_vel_topic:=/parrot/cmd_vel


# Step 3
# Terminal 3

# Just Rover
source /opt/ros/humble/setup.bash
source ~/git/TrailBotPro/install/setup.bash

ros2 topic pub --once /mission/target geometry_msgs/msg/PoseStamped \
"{header: {frame_id: \"odom\"}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# Just the drone
source /opt/ros/humble/setup.bash
source ~/git/TrailBotPro/install/setup.bash
ros2 run 41068_ignition_bringup mission_interface