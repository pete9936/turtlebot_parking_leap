#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from leap_motion.msg import leapros

class GoForward():
    def __init__(self):
        # initiliaze
        rospy.init_node('GoForward', anonymous=False)

	# tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")
        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)

	# Create a subscriber which can subscribe to LeapMotion roll-pitch-yaw data
	# Our listener aka subscriber. Listens to: leapmotion/data
   	rospy.Subscriber("leapmotion/data", leapros, self.callback_ros)
    	rospy.spin()

	# as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
	    # publish the velocity
            self.cmd_vel.publish(move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    # Callback of the ROS subscriber, just print the received data.

    def callback_ros(self,data):
##        rospy.loginfo(rospy.get_name() + ": Leap ROS Data %s" % data)
        print(data.ypr)
        move_cmd = Twist()
        # Forward/Backward movement
        if data.ypr.y >= 10.0 and data.ypr.y < 20.0:
            move_cmd.linear.x = -0.1

        elif data.ypr.y >= 20.0 and data.ypr.y < 45.0:
            move_cmd.linear.x = -data.ypr.y*0.01

        elif data.ypr.y >= 45.0:
            move_cmd.linear.x = -0.5

        elif data.ypr.y <= -10.0 and data.ypr.y > -20.0:
            move_cmd.linear.x = 0.1

        elif data.ypr.y <= -20.0 and data.ypr.y > -45.0:
            move_cmd.linear.x = data.ypr.y*0.01

        elif data.ypr.y <= -45.0:
            move_cmd.linear.x = 0.5

        else:
            move_cmd.linear.x = 0
	# Rotational movement
        if data.ypr.z >= 10.0 and data.ypr.z < 20.0: 
            print("TURN RIGHT")
            move_cmd.angular.z = 0.2

        elif data.ypr.z >= 20.0 and data.ypr.z < 45.0:
            print("TURN RIGHT")
            move_cmd.angular.z = data.ypr.z*0.01

        elif data.ypr.z <= -10.0 and data.ypr.z > -20.0:
            print("TURN LEFT")
            move_cmd.angular.z = -0.2

        elif data.ypr.z <= -20.0 and data.ypr.z > -45.0:
            print("TURN LEFT")
            move_cmd.angular.z = -data.ypr.z*0.01

        elif data.ypr.z <= -45.0:
            print("TURN LEFT")
            move_cmd.angular.z = -0.5
            
        else:
            move_cmd.angular.z = 0.0

	# Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.cmd_vel.publish(move_cmd)
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        GoForward()
    except:
        rospy.loginfo("GoForward node terminated.")
