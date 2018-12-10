#!/usr/bin/env python
import rospy
import os
import pickle
import numpy as np
import csv
from geometry_msgs.msg import Twist
from leap_motion.msg import leap
from leap_motion.msg import leapros
from std_msgs.msg import String
from geometry_msgs.msg import Point
from pybrain.tools.shortcuts import buildNetwork
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.datasets import SequentialDataSet, SupervisedDataSet,ClassificationDataSet
from pybrain.structure import SigmoidLayer, LinearLayer


data1=Point()
x=[]
y=[]
z=[]

mat=[]

# initiliaze
rospy.init_node('gesture_rec', anonymous=True)

def __init__(self):
    # tell user how to stop TurtleBot
    rospy.loginfo("To stop gesture recognition CTRL + C")
    # What function to call when you ctrl + c    
    rospy.on_shutdown(self.shutdown)



def callback_ros(data):
  global data1
  data1=data.palmpos

def subscriber():
  rospy.Subscriber("leapmotion/data", leapros, callback_ros)
  
subscriber()

pub = rospy.Publisher('/neuro_gest/gesture',String, queue_size=5)

f1=open('/home/pete9936/catkin_ws/src/neuro_gesture_leap/my_data_sets/pic1.pickle','rb')
trainer=pickle.load(f1)
n=pickle.load(f1)
gest_name=pickle.load(f1)

while not rospy.is_shutdown():
  x[:]=[]
  y[:]=[]
  z[:]=[]
  mat[:]=[]
  index=0
  time=rospy.get_time()
  while rospy.get_time()<time+3.5:
     x.append(data1.x)
     y.append(data1.y)
     z.append(data1.z)
     rospy.sleep(0.1)
  all_points=[x,y,z]
  for k in range(0,len(x)-1,1):
      mat.append(x[k+1]-x[k])
      mat.append(y[k+1]-y[k])
      mat.append(z[k+1]-z[k])
  out = n.activate(mat)
  index=np.argmax(out)
   
  print "The gesture is '"+gest_name[index]+"'"
  pub.publish(gest_name[index])
     


	# Create a publisher which can "talk" to TurtleBot and tell it to move
        # self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        # self.cmd_vel.publish(move_cmd)
        
def shutdown(self):
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    self.cmd_vel.publish(Twist())
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
    rospy.sleep(1)



