#!/usr/bin/env python

###################################################################
# CSCI 5551 Self-parking Turtlebot with Gesture Control project
# Jefferson Hui, Ryan Peterson, Syed Zohair Naqvi
# December 2018
# University of Minnesota
###################################################################


# stage 1 = automatic (drive in a straight line, adjust orientation based on AR Marker on the front wall)
# stage 2 = move to goal (after identifying an empty spot, the turtlebot moves to a point on the map to get ready for parallel parking)
# stage 3 = manual driving (use leap to drive the robot. This will activate when the robot reaches a certain distance from the front wall)
# stage 4 = parallel parking (shutdown): the robot follows the parallel parking trajectory into a valid parking spot

import rospy
from geometry_msgs.msg import Twist
from leap_motion.msg import leapros
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

global lastypr
lastypr = 0.0

class GoForward():
    def __init__(self):
	global stage
	global index
	stage = 1
	index = 0
   	rospy.Subscriber("leapmotion/data", leapros, self.leapCallback)
   	rospy.Subscriber("/neuro_gest/gesture", String, self.gestureCallback)
        rospy.Subscriber("/tvec1", Vector3, self.camera1Callback)
        rospy.Subscriber("/tvec2", Vector3, self.camera2Callback) 
	rospy.spin()

    def leapCallback(self, data):
	global stage
        global lastypr
	move_cmd = Twist()
	if stage == 3:
            if data.ypr.y == lastypr:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0

            else:
		# Forward/Backward movement
		if data.ypr.y >= 10.0 and data.ypr.y < 20.0:
		    move_cmd.linear.x = -0.1

		elif data.ypr.y >= 20.0 and data.ypr.y < 45.0:
		    move_cmd.linear.x = -0.25

		elif data.ypr.y >= 45.0:
		    move_cmd.linear.x = -0.35

		elif data.ypr.y <= -10.0 and data.ypr.y > -20.0:
		    move_cmd.linear.x = 0.1

		elif data.ypr.y <= -20.0 and data.ypr.y > -45.0:
		    move_cmd.linear.x = 0.25

		elif data.ypr.y <= -45.0:
		    move_cmd.linear.x = 0.35

		else:
		    move_cmd.linear.x = 0

		# Rotational movement
		if data.palmpos.x >= 40.0 and data.palmpos.x < 100.0: 
		    move_cmd.angular.z = -0.2

		elif data.palmpos.x >= 100.0 and data.palmpos.x < 180.0:
		    move_cmd.angular.z = -0.4

		elif data.palmpos.x >= 180.0:
		    move_cmd.angular.z = -0.8

		elif data.palmpos.x <= -40.0 and data.palmpos.x > -100.0:
		    move_cmd.angular.z = 0.2

		elif data.palmpos.x <= -100.0 and data.palmpos.x > -180.0:
		    move_cmd.angular.z = 0.4

		elif data.palmpos.x <= -180.0:
		    move_cmd.angular.z = 0.8
		    
		else:
		    move_cmd.angular.z = 0.0

		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        	self.cmd_vel.publish(move_cmd)
            lastypr = data.ypr.y

    def gestureCallback(self, gesturedata):
	global stage
	## global char
	move_cmd = Twist()
	if stage == 1:
                print(gesturedata.data)
		if gesturedata.data == 'Gandalf':   # Go into manual mode
			stage = 3
			print("Gandalf: Entering manual mode")
        		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        		self.cmd_vel.publish(move_cmd)
		else:
			stage = 1            
	if stage == 2:
                if gesturedata.data == 'Swipe':   # SKIP GESTURE
                        print("Swipe: Skipping spot")
            		stage = 1
        		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        		self.cmd_vel.publish(move_cmd)
		elif gesturedata.data == 'Gandalf':   # Go into manual mode
			stage = 3
			print("Gandalf: Entering manual mode")
        		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        		self.cmd_vel.publish(move_cmd)
		else:
			stage = 2

	if stage == 3:
## 		if char=="a" or char=="A":
##            		stage = 1
##        		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
##        		self.cmd_vel.publish(move_cmd)
##        		char = 'b'
        	if gesturedata.data == 'Handout':   # Go into autonomus mode
			stage = 1
			print("Handout: Entering Autonomous mode")
        		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        		self.cmd_vel.publish(move_cmd)
		else:
			stage = 3

    def camera1Callback(self, tvec1):   # straight camera
	global stage
	global index
	rvec1 = rospy.wait_for_message("/rvec1", Vector3)
	move_cmd = Twist()

	if stage == 1:
                print(tvec1.z)
		index = 0
		if tvec1.z > 1.5:
            		move_cmd.linear.x = 0.1
			if rvec1.z > 1.0: 
            			move_cmd.angular.z = 0.2
			else:
            			move_cmd.angular.z = 0.0				
        		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        		self.cmd_vel.publish(move_cmd)
        		print("GOING STRAIGHT")

        	else:
            		move_cmd.linear.x = 0.0
            		move_cmd.angular.z = 0.0
        		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        		self.cmd_vel.publish(move_cmd)
			stage = 3
			print("END OF WALL: SWITCH TO MANUAL MODE")

	elif stage == 2:
		if index < 10:
			move_cmd.linear.x = 0.1
			if rvec1.z > 1.0: 
            			move_cmd.angular.z = 0.0
			else:
            			move_cmd.angular.z = 0.0
        		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        		self.cmd_vel.publish(move_cmd)
        		rospy.sleep(.5)
        		print("READY FOR PARKING")
		else:
			self.shutdown()
			rospy.signal_shutdown("Backing In")
		index += 1


    def camera2Callback(self, tvec2):      # side camera
	global stage
	rvec2 = rospy.wait_for_message("/rvec2", Vector3)
	move_cmd = Twist()
	if stage == 1:
            if abs(tvec2.x) < .05:
		stage = 2
            	move_cmd.linear.x = 0.0
            	move_cmd.angular.z = 0.0
        	self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        	self.cmd_vel.publish(move_cmd)
		print("Parking Spot Detected")
	
#	elif stage == 2:
#            	move_cmd.linear.x = 0.1
#            	move_cmd.angular.z = 0.0
#        	self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
#        	self.cmd_vel.publish(move_cmd)


    def shutdown(self):
	move_cmd = Twist()
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	lin_vel_list = ([-0.15,-0.15,-0.1,-0.1,-0.1,-0.1,-.1,-.1,-.1,-.1,-.1,-.1,-.1,-.1,-.1,-.1,-.1,-.1,-.1,-.05,-.05,-.04,-.03,-.03,-.02])
	lin_vel_list = [x*1 for x in lin_vel_list]
	ang_vel_list = ([0.0,0.0,0.0,0.0,0.3,0.5,0.7,0.9,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.9,-0.7,-0.52,-0.25,0.0,0.0])
	ang_vel_list = [x*-1 for x in ang_vel_list]

	for i in range(len(lin_vel_list)):
		ang_vel = ang_vel_list[i]
		lin_vel = lin_vel_list[i]
        	move_cmd.linear.x = lin_vel
		move_cmd.angular.z = ang_vel
		self.cmd_vel.publish(move_cmd)
		rospy.sleep(0.5)
	print("JUST PARKED!")

if __name__ == '__main__':
    rospy.init_node('GoForward', anonymous=False)
    try:
	GoForward()
    except:
       	rospy.loginfo("GoForward node terminated.")

