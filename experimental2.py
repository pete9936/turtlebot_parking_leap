#!/usr/bin/env python

# stage 1 = automatic (drive in a straight line, adjust orientation based on AR Marker on the front wall)
# stage 2 = move to goal (after identifying an empty spot, the turtlebot moves to a point on the map to get ready for parallel parking)
# stage 3 = manual driving (use leap to drive the robot. This will activate when the robot reaches a certain distance from the front wall)
# stage 4 = parallel parking (shutdown): the robot follows the parallel parking trajectory into a valid parking spot

import rospy
from geometry_msgs.msg import Twist
from leap_motion.msg import leapros
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

class GoForward():
    def __init__(self):
	global stage
	stage = 1
	global index
	index = 0
   	rospy.Subscriber("leapmotion/data", leapros, self.leapCallback)
   	rospy.Subscriber("/neuro_gest/gesture", String, self.gestureCallback)
	char=raw_input("Enter 'a' to go into Autonomous mode: ")
        rospy.Subscriber("/tvec1", Vector3, self.camera1Callback)
        rospy.Subscriber("/tvec2", Vector3, self.camera2Callback) 
	rospy.spin()

    def leapCallback(self, leapdata):
	global stage
	move_cmd = Twist()
	if stage == 3:
		# Forward/Backward movement
		if data.ypr.y >= 10.0 and data.ypr.y < 20.0:
		    move_cmd.linear.x = -0.1

		elif data.ypr.y >= 20.0 and data.ypr.y < 45.0:
		    move_cmd.linear.x = -0.3

		elif data.ypr.y >= 45.0:
		    move_cmd.linear.x = -0.5

		elif data.ypr.y <= -10.0 and data.ypr.y > -20.0:
		    move_cmd.linear.x = 0.1

		elif data.ypr.y <= -20.0 and data.ypr.y > -45.0:
		    move_cmd.linear.x = 0.3

		elif data.ypr.y <= -45.0:
		    move_cmd.linear.x = 0.5

		else:
		    move_cmd.linear.x = 0

		# Rotational movement
		if data.ypr.z >= 10.0 and data.ypr.z < 20.0: 

		    move_cmd.angular.z = 0.2

		elif data.ypr.z >= 20.0 and data.ypr.z < 45.0:
		    move_cmd.angular.z = 0.35

		elif data.ypr.z >= 45.0:
		    move_cmd.angular.z = 0.5

		elif data.ypr.z <= -10.0 and data.ypr.z > -20.0:
		    move_cmd.angular.z = -0.2

		elif data.ypr.z <= -20.0 and data.ypr.z > -45.0:
		    move_cmd.angular.z = -0.35

		elif data.ypr.z <= -45.0:
		    move_cmd.angular.z = -0.5
		    
		else:
		    move_cmd.angular.z = 0.0

		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        	self.cmd_vel.publish(move_cmd)

    def gestureCallback(self, gesturedata):
	global stage
	move_cmd = Twist()
	a = 1
	if stage == 2:
                if a == 5:
		if gesturedata == 'Swipe':   # SKIP GESTURE
            		stage = 1
        		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        		self.cmd_vel.publish(move_cmd)
		elif gesturedata == 'Gandolf':   # Go into manual mode
			stage = 3
        		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        		self.cmd_vel.publish(move_cmd)
		else:
			stage = 2

	if stage == 3:
 		if char=="a" or char=="A":
            		stage = 1
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
		index = 0
		if tvec1.z > .4:
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
		if index < 6:
			move_cmd.linear.x = 0.1
			if rvec1.z > 1.0: 
            			move_cmd.angular.z = 0.0
			else:
            			move_cmd.angular.z = 0.0
        		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        		self.cmd_vel.publish(move_cmd)
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
	ang_vel_list = [x*1 for x in ang_vel_list]

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


