# turtlebot_parking_leap
========================

Repository for a turtlebot to autonomously park while also having human gesture input using a Leap Motion. Using ROS Kinetic running Linux 16.04.

Hardware required: Turtlebot with its own laptop, two depth cameras (we used Astra) Kinect would also work, and a LeapMotion for gesture detection.

## Install LeapMotion Packages
First create a ROS workspace for the project, for this I am calling it 'catkin_ws':
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

For the leap motion first clone the LeapMotion package we modified for ROS (this has the LeapSDK included) into your newly created catkin_ws: 
```
cd ~/catkin_ws/src
git clone https://github.com/pete9936/leap_motion
cd ..
catkin_make
```

For using gesture control we used the neuro_gesture_leap package in order to create unique gestures that could easily be recognized and used for high level control. All of the details including where to download the repository and how to use the code to create your own gestures can be found at the link below:
http://wiki.ros.org/TanvirParhar/neuro_gesture_leap 

We changed the record time of the gestures to 2.0 seconds (currently 3.5 in package). This allows for a more fluid real-time gesture command for the Turtlebot. To train gestures accordingly replace the get_gesture.py file with our get_gesture.py file. You will also need to replace the gesture_rec.py file with our gesture_rec.py file.
In order to record gestures on a continual 2 second feed without manual input use the gesture_rec2.py file.

To test the leap_motion first make sure the LeapMotion is running (sudo leapd) or (sudo service leapd restart) and to visulaize the LeapMotion (in a new terminal) type "LeapControlPanel" then bring up the Visualizer. Start roscore then run leap_turtle.launch (this launches both the sender.py file and gesture_rec2.py).

## Remotely connecting to Turtlebot
To connect to the Turtlebot and run everything properly you will have to set up a few parameters in your .bashrc. Add the following lines with the proper ip addresses for both your remote work station and the turtlebot laptop:
```
export ROS_MASTER_URI=http://localhost:11311
export ROS_MASTER_URI=http://IP.ADDRESS.TURTLE.BOT  # turtlebot IP address
export ROS_HOSTNAME=IP.ADDRESS.LOCAL.MACHINE  # your laptop IP address
```

## Setting up the Turtlebot laptop

In the .bashrc

## Get necessary imaging toolboxes

To set up the Astra follow the directions to get the astra_camera and astra_launch packages found below:
http://wiki.ros.org/Sensors/OrbbecAstra 

This project was verified using OpenCV 3.3.1, though other versions will most likely work fine.

In order to identify a parking space we used the Astra camera to detect a CV code (similar to AR code) which when detected by the camera would initiate an autonomous parking maneuver.

For the full navigation scheme use experimental2.py to run the overall architecture for the Turtlebot. Everything can be launched from "..."
