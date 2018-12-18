# turtlebot_parking_leap
========================

Repository for a turtlebot to autonomously park while also having human gesture input using a Leap Motion. Using ROS Kinetic running Linux 16.04.

Hardware required: Turtlebot with its own laptop, two depth cameras (we used Astra) Kinect would also work, and a LeapMotion for gesture detection.

Details on our method and overall project can be found in our Project Report ```CSCI_5551_Final_Report.pdf``` given above, and a demonstration is given in the video at this link: https://drive.google.com/file/d/1lZUKtvfeqe9mcH4OGtyH4-FplmpjByGg/view?usp=sharing

## Installation
First create a ROS workspace for the project, for this I am calling it 'catkin_ws':
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
Now that we have our workspace created we can add this repository and the LEAP motion repository we modified. This has the LeapSDK included. clone this repository into your ```catkin_ws/src``` with the following:

```
cd ~/catkin_ws/src
git clone https://github.com/pete9936/turtlebot_parking_leap.git
git clone https://github.com/pete9936/leap_motion
cd ..
catkin_make
```
If catkin_make gives you errors, download the required depencies.

Now we should have the ```turtlebot_parking_leap``` and ```leap_motion``` repositories installed. Now let's first try out the leap_motion.

### Running the LEAP motion
To test the LEAP motion first make sure the LeapMotion is running:
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
All of our modifications including our dataset can be found in the ```neuro_gest_files``` folder. Simply move these into the neuro_gesture_leap repository to use.

We found that using 15 training sets works well, but if you desire more sensitive gesture detection use more training sets. We used 30 for each gesture, which maked gesture detection a lot more sensitive. This was desirable in our case. All of the trained data and data sets for our gestures are located in the "my_data_sets" folder.

We changed the record time of the gestures to 2.0 seconds (currently 3.5 in package). This allows for a more fluid real-time gesture command for the Turtlebot. To train gestures accordingly replace the ```get_gesture.py``` file from the TanvirParhar repository with our ```get_gesture_2sec.py``` file. You will also need to replace ```gesture_rec.py``` with our ```gesture_rec_2sec.py``` file.

In order to record gestures on a continual 2.0 second feed without manual input use the ```gesture_rec2.py``` file when running the Leap Motion.

Now we can test this out. First start roscore then in separate terminals run the sender.py file and the gesture_rec.py (if you just want to record one gesture at a time) or gesture_rec2.py (if you want to record gestures continually).
```
rosrun leap_motion sender.py
```
Then in a new terminal run:
```
rosrun leap_motion gesture_rec2.py
```
Make sure that both files are executables (shows up green in terminal), if they are not then make them executables. To make a python file an executable, navigate to the directory that the file is located in and use the command:
```
chmod +x <file_name>.py
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
rosrun robot detect_markers1 -c="<path_to_camera_intrinsic_file_1>.yml" -d=5 -ci=1 -l=.196
rosrun robot detect_markers2 -c="<path_to_camera_intrinsic_file_2>.yml" -d=5 -ci=2 -l=.196
```

For the full navigation scheme use experimental2.py to run the overall architecture for the Turtlebot.



## Running Everything
This project requires 3 USB ports for the Turtlebot laptop. Connect the Turtlebot to 1 USB port and the 2 Astra cameras to 2 other USB ports.

For Turtlebot:

On the Turtlebot laptop start roscore in terminal. 
```
roscore
```
In a new terminal tab, execute the following lines:
```
echo export ROS_MASTER_URI=http://localhost:11311 >> ~/.bashrc
echo export ROS_HOSTNAME=IP_OF_TURTLEBOT >> ~/.bashrc
```
In the current tab execute this line:
```
rosrun robot detect_markers1 -c="<path_to_camera_intrinsic_file_1>.yml" -d=5 -ci=1 -l=.196
```
Open a new terminal tab and execute this line:
```
rosrun robot detect_markers1 -c="<path_to_camera_intrinsic_file_2>.yml" -d=5 -ci=2 -l=.196

```

For workstation:

On the remote workstation connect to the turtlebot via ssh and type turtlebot password when prompted:
```
ssh <username>@<turtlebot ip address>
```
You must also connect your workstation to the same ROS network as the turtlebot laptop. Make sure your workstation is connected to the same WiFi network as the Turtlebot laptop. On your remote workstation terminal, execute the following lines:
```
echo export ROS_MASTER_URI=http://IP_OF_TURTLEBOT:11311 >> ~/.bashrc
echo export ROS_HOSTNAME=IP_OF_WORKSTATION >> ~/.bashrc
```
Now that remote station is connected, to actually send commands to the turtlebot you must first type the following command in the SSH'd terminal tab:
```
roslaunch turtlebot_bringup minimal.launch
```
For a quick verification that this in fact is working (in a new terminal) try controlling turtlebot with teleop first. You must open a new tab again and SSH into the Turtlebot.
```
ssh <username>@<turtlebot ip address>
roslaunch turtlebot_teleop keyboard_teleop.launch
```
Once this is verified to be working, hit Ctrl+C to close the teleop. Make sure you do this or the robot will not run the parallel parking functions.
To run the entire working robot algorithms open each of the following commmands in a NEW terminal tab:
```
sudo leapd
LeapControlPanel
rosrun leap_motion leap_turtle.launch
```
Open another new terminal tab SSH into the turtlebot again in this tab:
```
ssh <username>@<turtlebot_ip_address>
```
Navigate to the turtleleapbot_main.py directory and execute turtleleapbot_main.py:
```
cd ~/catkin_ws/src/turtlebot_parking_leap/src/
python turtleleapbot_main.py
```

Now the Turtlebot should run our Self-parallel Parking with Gesture Control algorithm!
