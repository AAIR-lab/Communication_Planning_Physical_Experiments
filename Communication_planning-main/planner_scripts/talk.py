#!/usr/bin/env python

import rospy
import os 
from playsound import playsound
from communication_planning.srv import  Talk, TalkResponse

class Robot_Mouth(object):  

    def __init__(self):
        rospy.init_node('robot_mouth')
        self.service_mouth = rospy.Service('robot_mouth', Talk, self.handle_request_talk)
        self.rate = rospy.Rate(10)
        rospy.spin()


    def handle_request_talk(self, req): #this service provides the current pose of the human and the robot
        response = TalkResponse()
        file_name = ""
        if req.signal == "east":
            file_name = "tv.mp3"
        elif req.signal == "west":
            file_name = "desks.mp3"
        elif req.signal == "north":
            file_name = "couch.mp3"
        elif req.signal == "south":
            file_name = "sink.mp3"

        mp3_path = os.path.dirname(os.path.realpath(__file__)) + "/" + "audio/" + file_name
        if file_name != "": playsound(mp3_path)
        return response


if __name__ == '__main__':
    r_mouth = Robot_Mouth()


