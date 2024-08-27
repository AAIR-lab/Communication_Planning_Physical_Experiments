#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String
import os
import json


dir_path = os.path.dirname(os.path.realpath(__file__))
dir_jason_path = dir_path

class Path_Publisher():
     
    def __init__(self):
        rospy.init_node('plan_publisher', anonymous=True)
        self.publisher = rospy.Publisher('/to_go', PoseArray, queue_size=1)
        self.rate = rospy.Rate(1) 
    
    
    def read_object_file (self):
        with open (dir_jason_path + "/to_go_path.json") as in_file:
            self.path = json.load (in_file)

    def path_2_arraypose (self, path):
        posearray = PoseArray()
        posearray.header.frame_id = 'base_link'
        for point in path:
            pose = Pose()
            pose.position.x = point [0]
            pose.position.y = point[1]
            pose.orientation.z = point[2] 
            posearray.poses.append (pose)
        return posearray


    def publish_waypoints (self):
        while not rospy.is_shutdown():
            self.read_object_file()
            posearray = self.path_2_arraypose(self.path)
            self.publisher.publish(posearray)
            self.rate.sleep()


if __name__ == "__main__":
    pb = Path_Publisher()
    try:
        pb.publish_waypoints()
    except:
        print ("error")
        
