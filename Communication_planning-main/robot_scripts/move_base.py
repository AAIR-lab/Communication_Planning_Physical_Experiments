#!/usr/bin/env python

import actionlib
import rospy
import copy
import math
import numpy as np
from math import sin, cos
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray
from communication_planning.srv import RobotInfo, RobotInfoResponse, LiveLocation, LiveLocationRequest, Transforms, TransformsRequest
from tf.transformations import quaternion_from_euler


# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        rospy.init_node("move_cobot")
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.service_robot_info = rospy.Service('robot_info', RobotInfo, self.handle_request_robot_info)
        self._moving_flag = False
        rospy.wait_for_service('live_location')
        self.service_live_locations = rospy.ServiceProxy('live_location', LiveLocation)
        rospy.wait_for_service('transforms')
        service_transforms = rospy.ServiceProxy('transforms', Transforms)
        transforms_info = service_transforms()
        self.angle = transforms_info.angle
        self.disp = transforms_info.disp
        self._dist_threshold = 0.2

    def handle_request_robot_info (self, req):
        response = RobotInfoResponse ()
        if req.info == "robot_moving":
            response.is_moving = self._moving_flag
        return response

    def move(self, data):
        print ("received a path from cobot_planner")
        poses = data.poses
        for p in poses:
            self._moving_flag = True
            move_goal = MoveBaseGoal()
            ori_quaternion = quaternion_from_euler(0,0,p.orientation.z - np.pi)
            move_goal.target_pose.pose.position.x = p.position.x
            move_goal.target_pose.pose.position.y = p.position.y
            move_goal.target_pose.pose.orientation.x = ori_quaternion[0]
            move_goal.target_pose.pose.orientation.y = ori_quaternion[1]
            move_goal.target_pose.pose.orientation.z = ori_quaternion[2]
            move_goal.target_pose.pose.orientation.w = ori_quaternion[3] 
            move_goal.target_pose.header.frame_id = "map"
            move_goal.target_pose.header.stamp = rospy.Time.now()
            self.client.send_goal(move_goal)
            #self.wait_for_results (move_goal)
            #self.client.send_goal_and_wait(move_goal)
            rospy.Duration(0.2)
        self.client.wait_for_result()
        self._moving_flag = False
        print ("done with the path")

    def wait_for_results (self, move_goal):
            x = move_goal.target_pose.pose.position.x
            y = move_goal.target_pose.pose.position.y
            yaw = move_goal.target_pose.pose.orientation.w
            robot_goal = [x, y, yaw]
            robot_current = self.get_agents_location()
            while self.is_too_far(robot_goal, robot_current):
                robot_current = self.get_agents_location()


    def get_agents_location (self):
        robot = self.service_live_locations('robot')
        RL = [robot.location.position.x, robot.location.position.y, robot.location.orientation.z]
        RL = self.trasnform_reverse (RL)
        return RL
    
    def is_too_far (self, current, goal):
        current, goal = np.array(current), np.array(goal)
        distance = np.linalg.norm (current[0:2] - goal [0:2]) 
        if  distance > self._dist_threshold: return True
        else: return False

    

    def trasnform (self, point):      
        cos_angle = math.cos(-self.angle)
        sin_angle = math.sin(-self.angle)
        transformed_x = point[0] * cos_angle - point[1] * sin_angle + self.disp 
        transformed_y = point[0] * sin_angle + point[1] * cos_angle + self.disp
        if len(point) > 2:
            transformed_yaw = point[2] - self.angle
            return [transformed_x, transformed_y, transformed_yaw]
        else:
            return  [transformed_x, transformed_y]

    def trasnform_reverse (self, input_point):
        transformed_point = copy.deepcopy (input_point)      
        cos_angle = math.cos(self.angle)
        sin_angle = math.sin(self.angle)
        transformed_point [0], transformed_point [1] = transformed_point[0] - self.disp, transformed_point [1] - self.disp
        point_x = transformed_point[0] * cos_angle - transformed_point[1] * sin_angle
        point_y = transformed_point[0] * sin_angle + transformed_point[1] * cos_angle
        if len(transformed_point) > 2:
            point_yaw = transformed_point[2] + self.angle
            return [point_x, point_y, point_yaw]
        else: return [point_x, point_y]

if __name__ == "__main__":
    move_base = MoveBaseClient()
    rospy.Subscriber ("/to_go", PoseArray, move_base.move)
    rospy.spin()


    
