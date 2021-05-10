# Gazebo

How to run Gazebo for blind_test

Step 1 :(Get map)
  roslaunch blind_test gazebo_map.launch
  
Step 2 :(Get robot to gazebo)
  roslaunch blind_test spawn_moby.launch
  
Step 3 :(use teleop for move)
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
velocity max is 0.05!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
