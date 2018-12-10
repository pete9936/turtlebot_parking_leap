# turtlebot_parking_leap
Repository for a turtlebot to autonomously park while also having human gesture input using a Leap Motion. ROS Kinetic was used running Linux 16.04 for this project.

Hardware required: Turtlebot with its own laptop, two depth cameras (we used Astra) Kinect would also work, and a LeapMotion (detects your hands)

For the leap motion first download the LeapMotion package for ROS (this has the LeapSDK included):
https://github.com/ros-drivers/leap_motion

For using gesture control we used the neuro_gesture_leap package in order to create unique gestures that could easily be recognized and used for high level control. All of the details including where to download the repository and how to use the code to create your own gestures can be found at the link below:
http://wiki.ros.org/TanvirParhar/neuro_gesture_leap 

You suck!
