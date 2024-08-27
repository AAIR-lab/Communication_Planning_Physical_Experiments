#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from communication_planning.srv import AgentsGoals, AgentsGoalsRequest, LiveLocation, LiveLocationRequest, RobotInfo, RobotInfoRequest, Talk, TalkRequest



rospy.wait_for_service('live_location')
service_live_locations = rospy.ServiceProxy('live_location', LiveLocation)
rospy.wait_for_service('robot_info')
service_robot_info = rospy.ServiceProxy('robot_info', RobotInfo)
rospy.wait_for_service('robot_mouth')
service_robot_mouth = rospy.ServiceProxy('robot_mouth', Talk)

def is_robot_moving ():
    respose = service_robot_info ("robot_moving")
    robot_is_moving = respose.is_moving
    return robot_is_moving


def path_2_arraypose (path):
    posearray = PoseArray()
    posearray.header.frame_id = 'map'
    for point in path:
        pose = Pose()
        pose.position.x = point [0]
        pose.position.y = point[1]
        pose.orientation.z = point[2] 
        posearray.poses.append (pose)
    return posearray

def create_publisher ():
    pub = rospy.Publisher('/to_go', PoseArray, queue_size=5)
    return pub

def send_waypoints_to_robot (pub, path):
    posearray = path_2_arraypose(path)
    rate = rospy.Rate(1) 
    pub.publish(posearray)
    rate.sleep()

def get_agents_goals ():
    rospy.wait_for_service('agents_goals')
    service = rospy.ServiceProxy('agents_goals', AgentsGoals)
    robot_goal = service('robot')
    human_goal = service('human')
    RG = [robot_goal.location.position.x, robot_goal.location.position.y, 0]
    HG = [human_goal.location.position.x, human_goal.location.position.y] 
    return [RG], HG


def get_agents_location ():
    robot = service_live_locations('robot')
    human = service_live_locations('human')
    RL = [robot.location.position.x, robot.location.position.y, robot.location.orientation.z]
    HL = [human.location.position.x, human.location.position.y]
    return RL, HL, human.human_visible

def wait_for_move_base (r_start, h_start, human_visible):
    robot_is_moving = True
    robot_waypoints = []
    human_waypoints = []
    human_visibility = []
    r_start = r_start[0:2]
    r_start.append('')
    robot_waypoints.append(r_start)
    human_waypoints.append(h_start)
    human_visibility.append(human_visible)

    r_l, h_l, vis_flag = get_agents_location()
    r_l = r_l[0:2]
    r_l.append('')
    robot_waypoints.append(r_l)
    human_waypoints.append(h_l)
    human_visibility.append(vis_flag)

    while (not robot_is_moving):
        robot_is_moving = is_robot_moving()

    print ("robot is moving...")
    while (robot_is_moving):
        print (robot_is_moving)
        robot_is_moving = is_robot_moving()
        print (robot_is_moving)
        r_l, h_l, vis_flag = get_agents_location()
        r_l = r_l[0:2]
        r_l.append('')
        robot_waypoints.append(r_l)
        human_waypoints.append(h_l)
        human_visibility.append(vis_flag)
    
    r_l_new, h_l, vis_flag = get_agents_location()
    r_l = r_l_new[0:2]
    r_l.append('')
    robot_waypoints.append(r_l)
    human_waypoints.append(h_l)
    human_visibility.append(vis_flag)
    
    print ("robot finished the path!")
    print ("robot waypoints _________________________________________________________")
    print (robot_waypoints)
    print ("human waypoints _________________________________________________________")
    print (human_waypoints)
    return robot_waypoints, human_waypoints, human_visibility, r_l_new

def communicate_with_human (signal):
    service_robot_mouth(signal)
  





