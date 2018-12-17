#!/usr/bin/env python
import os
import rospy
import pickle
import csv
from leap_motion.msg import leap
from leap_motion.msg import leapros
from geometry_msgs.msg import Point

from pybrain.tools.shortcuts import buildNetwork
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.datasets import SequentialDataSet, SupervisedDataSet,ClassificationDataSet
from pybrain.structure import SigmoidLayer, LinearLayer

import numpy as np
rospy.init_node('get_gesture', anonymous=True)

dir_name=raw_input('Please put the directory of the folder where you would like to store the datasets :')

data1=Point()
x=[]
y=[]
z=[]
gest_name=[]
n_out=input('Please type the number of gestures:')
n_train_ex =input('Please type the number of training examples you want to give (10 to 15 would be a reasonable chioce):')

def print_sub():
  time_now=rospy.get_time()
  while rospy.get_time<time_now+2.5 :
   print "Hello"

def callback_ros(data):
 #rospy.loginfo(rospy.get_name() + ": Leap ROS Data %s" % data)
 global data1
 data1=data.palmpos
def subscriber():
  
  rospy.Subscriber("leapmotion/data", leapros, callback_ros)
  
subscriber()

while True:
  gest_name[:]=[]
  for i in range(0,n_out,1):
   
   gesture=raw_input("please enter the name for gesture number "+str(i+1)+" : ")
   gest_name.append(gesture)
   
  print "The gestures are :"
  print gest_name
  fine=raw_input("is it fine? (Y/N) ")
  if fine=="y" or fine=="Y":
    break
print "OK! The gestures are :"
print gest_name

for l in range(0,len(gest_name),1):
 for k in range(0,n_train_ex,1):
  x[:]=[]
  y[:]=[]
  z[:]=[]
  m=raw_input("For entering dataset number "+str(k+1)+"  for '"+ gest_name[l]+"' enter either 'm' or 'M':")
  if m=="m" or m=="M":
   time=rospy.get_time()
   while rospy.get_time()<time+2.0:
     print data1
     x.append(data1.x)
     y.append(data1.y)
     z.append(data1.z)
     rospy.sleep(0.1)
   all_points=[x,y,z]
   inputs=(len(x)-1)*3
   file_name=gest_name[l]+str(k+1)
   full_file_name=os.path.join(dir_name,file_name+'.csv')
   with open(full_file_name, 'w') as fp:
    a = csv.writer(fp, delimiter=',')
    a.writerows(all_points) 
    
print "OK! The data has been recorded. Just give me a moment to save the variables"
#path=raw_input("Please enter the path to your package : ")
pickle_file=os.path.join(dir_name,'pic.pickle')
f=open(pickle_file,'wb')
pickle.dump(dir_name,f)
pickle.dump(gest_name,f)
pickle.dump(n_train_ex,f)
pickle.dump(n_out,f)
pickle.dump(inputs,f)






