# fetch_follower

To use this package please make sure you have the following packages:
+ turtlebot3: https://github.com/ROBOTIS-GIT/turtlebot3
+ turtlebot3 gazebo: https://github.com/ROBOTIS-GIT/turtlebot3_simulations
+ fetch gazebo: https://github.com/fetchrobotics/fetch_gazebo
+ vision visp: http://wiki.ros.org/vision_visp
+ usb_cam: https://github.com/ros-drivers/usb_cam

----

Please put the file in the launch folder of this repo to the launch folder of fetch_gazebo and modify the cmakelist of fetch_gazebo to include the turtlebot3_gazebo as a dependency.

Please replace the turtlebot_description folder in the turtlebot3 package with the turtlebot_description folder in this repo.

For this project, to teleop the turtlebot, please modify the source code in the node folder of the turtlebot3_teleop so that it subcribes to "/tb3/odom" instead of "/odom" and publishes to "/tb3/cmd_vel" instead of "/cmd_vel".

Please replace the file name.cpp in the src folder of the visp_auto_tracker package with the name.cpp in the name_visp_auto_tracker folder of this repo.

----

To start the project: roslaunch fetch_gazebo fetch_purepursuit.launch

Wait for the gazebo simulation to start and the fetch and the turtlebot appear, then launch the visp node: roslaunch visp_auto_tracker tracklive_usb.launch

Launch the teleop node: roslaunch turtlebot3_teleop turtlebot3_teleop.launch

Finally launch the pure pursuit: rosrun fetch_purepursuit fetch_purepursuit
