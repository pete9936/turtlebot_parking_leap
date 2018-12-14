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
To test the leap_motion first make sure the LeapMotion is running:
```
sudo leapd
```
If that doesn't work try:
```
sudo service leapd restart
```
If neither of these work, open a new terminal and try again.

Now, to visualize the Leap Motion, in a new terminal bring up the leap control panel with command:
```
LeapControlPanel
```
Then bring up the visualizer, found in drop down menu of Leap Control Panel.

### Install gesture package for Leap Motion
For using gesture control we used the neuro_gesture_leap package in order to create unique gestures that could easily be recognized and used for high level control. All of the details including where to download the repository and how to use the code to create your own gestures can be found at the link below:
http://wiki.ros.org/TanvirParhar/neuro_gesture_leap 

For reference we show the updates made to our .bashrc below as this may be a little ambiguous in their documentation (note that I have pybrain cloned to my Desktop, not the home directory as they do):
```
export PYTHONPATH=~/Desktop/pybrain:$PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$~/catkin_ws/src/leap_motion/LeapSDK/lib:$~/catkin_ws/src/leap_motion/LeapSDK/lib/x64
```
In the "Making and Training Datasets" section they recommend 10-15 sets for each gesture. We found that using 15 works well, but if you desire more sensitive gesture detection use more training sets. We used 30 for each gesture, which maked gesture detection a lot more sensitive. This was desirable in our case. All of the trained data and data sets for our gestures are located in the "my_data_sets" folder.

We changed the record time of the gestures to 2.0 seconds (currently 3.5 in package). This allows for a more fluid real-time gesture command for the Turtlebot. To train gestures accordingly replace the ```get_gesture.py``` file from the TanvirParhar repository with our ```get_gesture.py``` file. You will also need to replace ```gesture_rec.py``` with our ```gesture_rec.py``` file.
In order to record gestures on a continual 2.0 second feed without manual input use the ```gesture_rec2.py``` file when running the Leap Motion.

Now we can test this out. First start roscore then in separate terminals run the sender.py file and the gesture_rec.py (if you just want to record one gesture at a time) or gesture_rec2.py (if you want to record gestures continually).
```
rosrun leap_motion sender.py
```
Then in a new terminal run:
```
rosrun leap_motion gesture_rec2.py
```
Make sure that both files are executables (shows up green in terminal), if they are not then make them executables, for example navigate to the folder and run chmod command:
```
cd ~/catkin_ws/src/neuro_gesure_leap/scripts
chmod +x gesture_rec2.py
```

### What our gestures do
* If the turtlebot has detected a parking spot we use the 'Swipe' gesture to tell the turtlebot to skip that parking spot and keep moving along autonomously.
* While turtlebot is in autonomous mode the 'Gandalf' gesture switches turtlebot into manual mode.
* While turtlebot is in manual mode we control linear velocity with hand pitch angle and angular velocity with hand x-position (if you are facing your laptop, hand moves side-to-side, parallel with screen). These are the following topics published from the LeapMotion:
  - linear velocity: angular.z
  - angular velocity: palmpos.z

## Remotely connecting to Turtlebot
To connect to the Turtlebot and run everything properly you will have to set up a few parameters in your .bashrc. Add the following lines with the proper ip addresses for both your remote work station and the turtlebot laptop:
```
export ROS_MASTER_URI=http://localhost:11311
export ROS_MASTER_URI=http://<turtlebot ip address>  # turtlebot IP address
export ROS_HOSTNAME=<remote machine ip address>  # your laptop IP address
```

## Setting up the Turtlebot laptop

For details on the installation of the ROS software for the turtlebot netbook go to the wiki page: http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation

Since the wiki page is meant for ROS indigo, (not kinetic which we are using) we have modified the input and conveniently appended it below, also we found that the 'rocon' packages to not install so we just removed them:
```
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-indigo-ar-track-alvar-msgs
```

In the .bashrc of the turtlebot add the following lines:
```
export ROS_MASTER_URI=http://<turtlebot ip address>:11311
export ROS_IP=<turtlebot ip address>
```

## Get necessary imaging toolboxes

To set up the Astra camera clone the following ros_astra_launch repository (branch: upstream) from Clearpath Robotics in your catkin workspace source folder:
```
cd ~/catkin_ws/src
git clone https://github.com/tonybaltovski/ros_astra_launch.git --branch upstream
git clone https://github.com/tonybaltovski/ros_astra_camera.git --branch upstream
cd ..
catkin_make
```
This project was verified using OpenCV 3.3.1, which comes with ROS Kinetic, and OpenCV 3.4.3. 

In order to identify a parking space we used the Astra camera to detect an AR marker which when detected by the camera would initiate an autonomous parking maneuver. To generate the AR marker, follow the tutorial on https://docs.opencv.org/3.3.1/d5/dae/tutorial_aruco_detection.html and compile and run create_marker:
```
g++ -o create_marker create_marker.cpp `pkg-config opencv --cflags --libs`
./create_marker "d5_id10.jpg" -d=5 -id=10 -si=true -ms=800
```
Generate 2 AR markers: one for the side camera and one for the front camera. 

Then, calibrate both Astra cameras and generate 2 intrinsic parameter files by following this link:
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
Name the 2 intrinsic parameter files differently.

Then, run detect_markers1 and detect_markers2 through ROS. Replace the -c argument with the path to your camera intrinsic parameters file.
```
rosrun robot detect_markers1 -c="/home/blingshock/openCVtutorials/intrinsic1.yml" -d=5 -ci=1 -l=.196
rosrun robot detect_markers2 -c="/home/blingshock/openCVtutorials/intrinsic2.yml" -d=5 -ci=2 -l=.196
```

For the full navigation scheme use experimental2.py to run the overall architecture for the Turtlebot.



## Running Everything
Now that you hopefully have everything installed and setup the launch procedure is as follows.

On the turtlebot laptop start roscore.

On your remote work station connect to the turtlebot via ssh and type turtlebot password when prompted:
```
ssh <username>@<turtlebot ip address>
```
Now that remote station is connected, to actually send commands to the turtlebot you must first type the following command:
```
roslaunch turtlebot_bringup minimal.launch
```
For a quick verification that this in fact is working (in a new terminal) try controlling turtlebot with teleop first:
```
roslaunch turtlebot_teleop keyboard_teleop.launch
```

After verifying everything is running properly run leap_turtle.launch file (this runs sender.py, gesture_rec2.py, and experiment2.py):
```
rosrun leap_motion leap_turtle.launch
```
