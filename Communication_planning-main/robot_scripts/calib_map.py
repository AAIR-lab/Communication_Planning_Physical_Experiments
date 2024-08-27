#!/usr/bin/env python

import control_msgs.msg
import tf
import trajectory_msgs.msg
import rospy
import os
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs
from ar_track_alvar_msgs.msg import AlvarMarkers
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from communication_planning.srv import LiveLocation, LiveLocationResponse, Walls, WallsResponse, AgentsGoals, AgentsGoalsResponse, Transforms, TransformsResponse
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
import numpy as np
import math
import copy
from geometry_msgs.msg import PoseArray


class Human_scan(object):  

    def __init__(self):
        self._map = "hallway"
        rospy.init_node('calib_map_and_live_locations')
        self.service_locations = rospy.Service('live_location', LiveLocation, self.handle_request_live_locations)
        self.service_calib = rospy.Service('calibrated_walls', Walls, self.handle_request_walls)
        self.service_goals = rospy.Service('agents_goals', AgentsGoals, self.handle_request_goals)
        self.service_transforms = rospy.Service('transforms', Transforms, self.handle_request_transform)
        self.rate = rospy.Rate(10)
        self._human_found = False
        self._robot_pose = Pose()
        self._human_pose = Pose()
        self._walls = []
        self._points = []
        self.goals = []
        self._people = []
        self._n_points_walls = {"hallway": 8}
        self._hor_index = {"hallway": [0,2,4,6]}
        self._ver_index = {"hallway":[1,3,5,7]}
        self._distance_to_wall_filter = 0.2
        self._map_calibrated = False
        self._max_x = None
        self._max_y = None
        self._min_x = None
        self._min_y = None
        rospy.Subscriber ("/clicked_point", PointStamped, self.callback_collect_points)
        rospy.Subscriber ("/people_tracker_measurements", PositionMeasurementArray, self.callback_human_leg)
        #rospy.Subscriber ("odom", Odometry, self.callback_robot)
        #rospy.Subscriber ("/ar_pose_marker", AlvarMarkers, self.callback_human)
        rospy.spin()
        

# _____________ callback functions ____________ 
    def callback_human_leg (self, data):
        if self._map_calibrated:
            self._human_found = False
            self._people = []
            self._people = data.people
            self.get_human_loc()
            if self._human_found: 
                print ("Human visible")
            else: print("human not visible")

    def callback_collect_points (self, data):
        n = self._n_points_walls [self._map]
        if len(self._points) < n:
            x = data.point.x
            y = data.point.y
            self._points.append ([x,y])
        elif len(self.goals) < 2:
            x = data.point.x
            y = data.point.y
            self.goals.append ([x,y])
        if len(self._points) == n and len(self.goals) == 2: 
            print ("Walls and agent goals received successfully!")
            self._map_calibrated = True
            self.extract_walls()

    def callback_robot (self, data):
        self._robot_pose = data.pose.pose

    def callback_human (self, data):
        self._human_found = False
        if len(data.markers) > 0:
            for tag in data.markers:
                if tag.id == 6:
                    self._human_pose = tag.pose.pose
                    self._human_found = True
                    break
                else: self._human_found = False
        else: self.human_found = False
        if self._human_found: print ("Human visible")
        else: print("human not visible")

# _____________ request-handle functions ____________ 
    def handle_request_walls(self, req): #from env.py to get the walls info
        response = WallsResponse()
        response.arr = self.path_2_arraypose(self._points)
        return response
    
    def handle_request_transform(self, req): #from env.py to get the transform_to_planning_frame info
        response = TransformsResponse()
        response.angle = self._angle
        response.disp = self._disp
        return response

    def handle_request_live_locations(self, req): #this service provides the current pose of the human and the robot
        response = LiveLocationResponse()
        if req.agent == "robot": 
            response.location = self.get_robot_loc()
            response.human_visible = False
        elif req.agent == 'human':
            if self._human_found:
                response.location = self._human_pose
                response.human_visible = True
            else:
                response.location = Pose()
                response.human_visible = False
        return response

    def handle_request_goals(self, req):
        response = AgentsGoalsResponse()
        loc = Pose()
        if req.agent == "robot": 
            goal = self.transform(self.goals[1])
            loc.position.x = goal[0]
            loc.position.y = goal[1]
            response.location = loc
            
        elif req.agent == 'human':
            goal = self.transform(self.goals[0])
            loc.position.x = goal[0]
            loc.position.y = goal[1]
            response.location = loc
        return response

# _____________ transformation functions ____________ 
    def extract_walls(self):
        n = self._n_points_walls[self._map]
        for i in range (n-1):
            self._walls.append(np.array([self._points[i], self._points[i+1]]))
        self._walls.append(np.array([self._points[7], self._points[0]]))
        self.find_angle(self._walls[0])
        self.find_displacement (self._points)
        self._walls_before_transformed = copy.deepcopy(self._walls)
        for i in range (len(self._walls)):
            self._walls[i][0] = self.transform(self._walls[i][0])
            self._walls[i][1] = self.transform(self._walls[i][1])
        self._points = []
        for w in self._walls:
            self._points.append(w[0])
        self.max_min_x_y()
    
    def find_angle(self, ref_wall):
        start = ref_wall [0]
        end = ref_wall [1]
        dx = end[0] - start[0]
        dy = end[1] - start [1]
        self._angle = math.atan2(dy,dx) + np.pi
        print (self._angle)

    def find_displacement (self, points):
        minim = np.inf
        for p in points:
            if min(p) < minim:
                minim = min(p) 
        self._disp = 4*abs(minim)

    def transform (self, point):      
        cos_angle = math.cos(-self._angle)
        sin_angle = math.sin(-self._angle)
        transformed_x = point[0] * cos_angle - point[1] * sin_angle + self._disp 
        transformed_y = point[0] * sin_angle + point[1] * cos_angle + self._disp
        if len(point) > 2:
            transformed_yaw = point[2] + self._angle
            return [transformed_x, transformed_y, transformed_yaw]
        else:
            return  [transformed_x, transformed_y]

    def transform_reverse (self, input_point):
        transformed_point = copy.deepcopy (input_point)      
        cos_angle = math.cos(self._angle)
        sin_angle = math.sin(self._angle)
        transformed_point [0], transformed_point [1] = transformed_point[0] - self._disp, transformed_point [1] - self._disp
        point_x = transformed_point[0] * cos_angle - transformed_point[1] * sin_angle
        point_y = transformed_point[0] * sin_angle + transformed_point[1] * cos_angle
        if len(transformed_point) > 2:
            point_yaw = transformed_point[2] - self._angle 
            return [point_x, point_y, point_yaw]
        else: return [point_x, point_y]
    
    def transform_path (self, input_path):
        path = []
        for i in range (len(input_path)):
            path.append (self.transform (input_path[i]))
        return path

    def transform_reverse_path (self, path_input):
        path = []
        for i in range (len(path_input)):
            path.append(self.trasnform_reverse(path_input[i]))
        return path
    
# _____________ get human loc ____________ 

    def person_is_valid (self, person): #filter the false detetctions
        # this filter is map specific and currently is developed for hallway map only
        x, y = self.transform([person.pos.x, person.pos.y])
        valid_person = True
        vertical_walls_indices = self._ver_index [self._map]
        horizontal_walls_indices = self._hor_index [self._map]
        walls_raw = self._walls
        for i in vertical_walls_indices:
            wall = walls_raw[i]
            if abs(x - wall[0][0]) < self._distance_to_wall_filter or abs(x - wall[1][0]) < self._distance_to_wall_filter:
                max_y, min_y = max(  [wall[0][1] , wall[1][1] ]  ), min(  [wall[0][1] , wall[1][1] ]  )
                if y > min_y and y < max_y:
                    valid_person = False
                    return valid_person
        for i in horizontal_walls_indices:
            wall = walls_raw[i]
            if abs(y - wall[0][1]) < self._distance_to_wall_filter or abs(y - wall[1][1]) < self._distance_to_wall_filter:
                max_x, min_x = max(  [wall[0][0] , wall[1][0] ]  ), min(  [wall[0][0] , wall[1][0] ]  )
                if x > min_x and x < max_x:
                    valid_person = False
                    return valid_person
                
        if x > self._max_x or x < self._min_x or y > self._max_y or y < self._min_y:
            valid_person = False
            return valid_person
        
        if y < self._points [2][1]:
            if x < self._points[3][0] or x > self._points[5][0]:
                valid_person = False
                return valid_person


        print (x, y)
        print (self._max_x, self._min_x, self._max_y, self._min_y)
        return valid_person
    
    def get_human_loc (self):
        people_filtered = []
        for p in self._people:
            if self.person_is_valid(p):
                people_filtered.append (p)
        if len (people_filtered) == 0:
            self._human_pose = Pose()
        else:
            self._human_found = True
            detected_human = people_filtered[0]
            detected_human_loc = [detected_human.pos.x, detected_human.pos.y]
            detected_human_loc_transformed = self.transform (detected_human_loc) 
            h_pose = Pose()
            h_pose.position.x = detected_human_loc_transformed [0]
            h_pose.position.y = detected_human_loc_transformed [1]
            h_pose.position.z = 0
            self._human_pose = h_pose

    def get_robot_loc(self):
        listener = tf.TransformListener()
        listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        euler_angles = euler_from_quaternion(rot)
        x, y, yaw = trans[0], trans[1], euler_angles[2] + np.pi
        print ("robot yaw" + str(yaw))
        robot_loc_transformed = self.transform([x,y,yaw])
        robot_pose = Pose()
        robot_pose.position.x = robot_loc_transformed[0]
        robot_pose.position.y = robot_loc_transformed [1]
        robot_pose.orientation.z = robot_loc_transformed[2]
        return robot_pose
    
    def max_min_x_y (self):
        max_x, max_y = -np.inf, -np.inf
        min_x, min_y = np.inf, np.inf
        for p in self._points:
            p_x, p_y = p[0], p[1]
            if p_x > max_x: max_x = p_x
            if p_x < min_x: min_x = p_x
            if p_y > max_y: max_y = p_y
            if p_y < min_y: min_y = p_y
        self._max_x = max_x
        self._min_x = min_x
        self._max_y = max_y
        self._min_y = min_y
        

    def transfrom_pose (self, ref_pose, current_frame, target_frame):
        listener = tf.TransformListener()
        listener.waitForTransform(current_frame, target_frame, rospy.Time(), rospy.Duration(1.0))
        pose_stamped = PoseStamped()
        pose_stamped.pose = ref_pose 
        pose_stamped.header.frame_id = current_frame
        pose_stamped.header.stamp = rospy.Time(0)
        try:
            transformed_pose = listener.transformPose (target_frame, pose_stamped)
            return transformed_pose.pose
        except tf.Exception as ex:
            rospy.logwarn ("TF Exception: {}".format(ex))
            return None
        

    def path_2_arraypose (self, path):
        posearray = PoseArray()
        posearray.header.frame_id = 'base_link'
        for point in path:
            pose = Pose()
            pose.position.x = point [0]
            pose.position.y = point[1]
            posearray.poses.append (pose)
        return posearray
    
    

if __name__ == '__main__':
    t_scanner = Human_scan()