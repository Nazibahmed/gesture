#!/usr/bin/env python

import rospy
from std_msgs.msg import String

global confrimation_flag
confrimation_flag = False

def UserInputCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "User said: %s", data.data)
    pass

def GRCallback(data):
    if data.data == lower("yes") or data.data == lower("no"):
        confrimation_flag = True
        rospy.loginfo(rospy.get_caller_id() + "User gestured: %s", data.data)
    # https://answers.ros.org/question/56247/how-to-quitexit-from-a-subscriber/
    pass

def RasaCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "Rasa reply: %s", data.data)
    pass


def main_node():

    rospy.init_node('main_node', anonymous=True)

    rospy.Subscriber("user_input", String, UserInputCallback)
    rospy.Subscriber("gesture_recognition", String, GRCallback)
    rospy.Subscriber("rasa_response", String, RasaCallback)

    ## we need to add rasa post request here

    ## we need to add condition for confirmation here 

    ## we need to add condition if confrmation is recived from either nodes then de-subscribe
    rospy.spin()

if __name__ == '__main__':
    main_node()