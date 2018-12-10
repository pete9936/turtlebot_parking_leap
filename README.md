# turtlebot_parking_leap
Repository for a turtlebot to autonomously park while also having human gesture input using a Leap Motion. ROS Kinetic was used running Linux 16.04 for this project.

Hardware required: Turtlebot with its own laptop, two depth cameras (we used Astra) Kinect would also work, and a LeapMotion (detects your hands)

For the leap motion first download the LeapMotion package we modified for ROS (this has the LeapSDK included): https://github.com/pete9936/leap_motion

For using gesture control we used the neuro_gesture_leap package in order to create unique gestures that could easily be recognized and used for high level control. All of the details including where to download the repository and how to use the code to create your own gestures can be found at the link below:
http://wiki.ros.org/TanvirParhar/neuro_gesture_leap 

We changed the record time of the gestures to 2.0 seconds (currently 3.5 in package). This allows for a more fluid real-time gesture command for the Turtlebot. To train gestures accordingly replace the get_gesture.py file with our get_gesture.py file. You will also need to replace the gesture_rec.py file with our get_gesture2.py file.

To test the leap_motion first make sure the LeapMotion is running (sudo leapd) or (sudo service leapd restart) and to visulaize the LeapMotion in a new terminal type "LeapControlPanel" then bring up the Visualizer. Start roscore then run leap_turtle.launch

# Setting up the Turtlebot Laptop

To set up the Astra follow the directions to get the astra_camera and astra_launch packages found below:
http://wiki.ros.org/Sensors/OrbbecAstra 

Using OpenCV 3.3.1



For the full navigation scheme use experimental2.py to run the overall architecture for the Turtlebot
