#!/usr/bin/env python
import rospy
import os
import pickle
import numpy as np
import csv
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

rospy.init_node('gesture_rec', anonymous=True)

def callback_ros(data):
 #rospy.loginfo(rospy.get_name() + ": Leap ROS Data %s" % data)
 global data1
 data1=data.palmpos

def subscriber():
  
  rospy.Subscriber("leapmotion/data", leapros, callback_ros)
  
subscriber()

pub = rospy.Publisher('/neuro_gest/gesture',String)

f1=open('pic1.pickle','rb')
trainer=pickle.load(f1)
n=pickle.load(f1)
gest_name=pickle.load(f1)

while True:
 x[:]=[]
 y[:]=[]
 z[:]=[]
 mat[:]=[]
 index=0
 char=raw_input("Enter 'm' and start making a gesture. Or anything else to exit : ")
 if char=="m" or char=="M":
   time=rospy.get_time()
   while rospy.get_time()<time+2.0:
     print data1
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
   
   
   print index
   print out
   print "The gesture is '"+gest_name[index]+"'"
   pub.publish(gest_name[index])
     
 else:
  print "See you soon!!!"
  break






