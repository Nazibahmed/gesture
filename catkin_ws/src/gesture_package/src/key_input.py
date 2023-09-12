#!/usr/bin/env python
import rospy
from rospy import * 
from std_msgs.msg import String

if __name__ == '__main__':
  
  publisher = rospy.Publisher('user_input', String, queue_size=10)
  init_node('user_input')
  
  print("Please enter your input here:\n")

  while not is_shutdown():

    something = input()
    print("\nYou said:" + something)

    publisher.publish(something)          
    rospy.sleep(1.0)