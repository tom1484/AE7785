Gazebo world for the final, from the course repo
git@github.gatech.edu:7785-Intro-to-Robotics-Research/Lab6_Gazebo.git (commit 8fd357d,
"Sign colors changed for Fall 2025"); clone kept in turtlebot3_ws/ext/Lab6_Gazebo.

- 7785_maze_signs.world = the repo's 7785_maze.world (robot with camera + 11 signs built in).
- models/ = the sign models. launch/sim.launch.py puts it on GAZEBO_MODEL_PATH, so nothing
  is copied into turtlebot3_gazebo or ~/.gazebo as the repo README suggests.
- The 7785_maze model (walls) is byte-identical to turtlebot3_gazebo's (Lab 5), so it is not copied.
- Camera: /simulated_camera/image_raw, 320x240 raw sensor_msgs/Image (rgb8), hfov 1.0856 rad, 10 Hz.
  The repo README says /camera/image_raw; that is wrong here. image_transport also offers .../compressed.
