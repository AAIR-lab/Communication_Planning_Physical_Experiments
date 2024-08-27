#!/usr/bin/env python

import numpy as np
from controller import controller_map1, controller_map2, controller_map3, controller_map4
from robot import robot_map1, robot_map2, robot_map3, robot_map4
from matplotlib import colors
import matplotlib.pyplot as plt
import rospy
from communication_planning.srv import Walls, WallsRequest, Transforms, TransformsRequest
import math
import copy
from people_msgs.msg import PositionMeasurementArray

class ENV:
    def __init__(self, map_name = None, res = 0.5, p_horizon = 3, human_start = [0,0], human_goal = [1,1],
                 human_traj = np.array([[7,0.0],[6.9,0.0],[6.8, 0.0]]), robot_start = [5.0, 0.0, 0.0], 
                 robot_goal = [[4.0, 0.0, 0.0]], r_radius = 0.5, goal_shift_radii = 1.5):
        self._map = map_name
        self._res = res
        self._walls = []
        rospy.init_node('env')
        rospy.wait_for_service('calibrated_walls')
        service = rospy.ServiceProxy('calibrated_walls', Walls)
        request = WallsRequest()
        request = self._map
        walls_raw = service(request)
        rospy.wait_for_service('transforms')
        service_transforms = rospy.ServiceProxy('transforms', Transforms)
        request = TransformsRequest()
        transforms_info = service_transforms()
        self.angle = transforms_info.angle
        self.disp = transforms_info.disp
        self._distance_to_wall_filter = 0.2
        
        if self._map == 'map1':
            self._name = 'map1'
            w = 5          # half-width of area
            h_width = 1.5    # horizontal corridor width
            v_width = 1.5    # vertical corridor width
            self._walls.append(np.array([[-w, w], [w, w]]))
            self._walls.append(np.array([[-w, -w], [w, -w]]))
            self._walls.append(np.array([[-w, -w], [-w, w]]))
            self._walls.append(np.array([[w, -w], [w, w]]))
            self._walls.append(np.array([[-w, h_width/2], [-v_width/2, h_width/2]]))
            self._walls.append(np.array([[-w, -h_width/2], [-v_width/2, -h_width/2]]))
            self._walls.append(np.array([[v_width/2, h_width/2], [w, h_width/2]]))
            self._walls.append(np.array([[v_width/2, -h_width/2], [w, -h_width/2]]))
            self._walls.append(np.array([[-v_width/2, -w], [-v_width/2, -h_width/2]]))
            self._walls.append(np.array([[v_width/2, -w], [v_width/2, -h_width/2]]))
            self._walls.append(np.array([[-v_width/2, h_width/2], [-v_width/2, w]]))
            self._walls.append(np.array([[v_width/2, h_width/2], [v_width/2, w]]))
            self._planning_horizon = p_horizon
            self._dt = 0.1
            self._env_bounds = type('', (), {})()
            self._env_bounds.y_min_map = -w              # [m]
            self._env_bounds.y_max_map = w               # [m]
            self._env_bounds.x_min_map = -w              # [m]
            self._env_bounds.x_max_map = w               # [m]
            self._env_bounds.y_max = h_width/2   # [m]
            self._env_bounds.y_min = -h_width/2  # [m]
            self._env_bounds.x_max = v_width/2   # [m]
            self._env_bounds.x_min = -v_width/2  # [m]
            self._goal_set = np.array(robot_goal)    # [x, y, theta]
            self._start = np.array(robot_start)      # [x, y, theta]
            self._human_goal = np.array(human_goal)
            self._human_obsv_traj = human_traj       # human observed trajectory
            # Human simulation parameters
            desired_force_factor = 2
            social_force_factor = 5.1
            obstacle_force_factor = 4
            obstacle_force_sigma = 0.1
            self._sf_params = [desired_force_factor,
                               social_force_factor,
                               obstacle_force_factor,
                               obstacle_force_sigma]

            # call the contoller and robot model of map1
            self._robot_model = robot_map1(self._dt, x = self._start, goal_set = self._goal_set, r = r_radius, goal_shift_radii=goal_shift_radii)
            lyp_info = self._robot_model.lyp_func()                # extract the lyapanove set info
            unsafe_info = self._robot_model.unsafe_func()                # extract the unsafe set info
            map_info = self._robot_model.map_func(self._env_bounds)            # extract the environment info
            self._controller= controller_map1(unsafe_info, map_info, lyp_info)          # controller object
            
            self._vis_bound_x_min = -w
            self._vis_bound_x_max = w
            self._vis_bound_y_min = -w
            self._vis_bound_y_max = w
            #___________________________________________________________________
            # Environment Grid Map
            self._x_bound = 2*w
            self._y_bound = 2*w
            self._x_len = int(self._x_bound/res)
            self._y_len = int(self._y_bound/res)
            self._y_offset = w  # to get non-negative coordinates
            self._x_offset = w  # to get non-negative coordinates
            self._maze = np.zeros((self._y_len+2,self._x_len+2))
            self._ox, self._oy = [], []
            self._human_loc = None
            self._robot_loc = None
            self._res = res


            a = self.where([-w,w])
            b = self.where([-v_width/2,h_width/2])
            for i in range (b[1],a[1]+1):
                for j in range (a[0],b[0]+1):
                    self._maze[i][j] = 1

            a = self.where([v_width/2,w])
            b = self.where([w,h_width/2])
            for i in range (b[1],a[1]+1):
                for j in range (a[0],b[0]+1):
                    self._maze[i][j] = 1

            a = self.where([-w,-h_width/2])
            b = self.where([-v_width/2,-w])
            for i in range (b[1],a[1]+1):
                for j in range (a[0],b[0]+1):
                    self._maze[i][j] = 1

            a = self.where([v_width/2,-h_width/2])
            b = self.where([w,-w])
            for i in range (b[1],a[1]+1):
                for j in range (a[0],b[0]+1):
                    self._maze[i][j] = 1

            # following lines add a frame to the grid map to facilitate path planning
            for i in range(0, self._x_len+2):
                self._maze[0][i] = 1

            for i in range(0, self._y_len+2):
                self._maze[i][self._x_len+2-1] = 1
        
            for i in range(0, self._x_len+2):
                self._maze[self._y_len+2-1][i] = 1
        
            for i in range(0, self._y_len+2):
                self._maze [i][0] = 1
            #_____________________________________________________________________________

        elif self._map == 'map2':
            self._name = 'map2'
            self._walls.append(np.array([[-4.1, -3.15], [1., -3.15]]))
            self._walls.append(np.array([[-4.1, -3.15], [-4.1, 2.6]]))
            self._walls.append(np.array([[1., -3.15], [1., 2.6]]))
            self._walls.append(np.array([[-4.1, 2.6], [1., 2.6]]))
            # center obstacle
            self._walls.append(np.array([[-1.35, -0.34], [-1., -0.34]]))
            self._walls.append(np.array([[-1.35, -0.34], [-1.35, 0.15]]))
            self._walls.append(np.array([[-1., -0.34], [-1., 0.15]]))
            self._walls.append(np.array([[-1.35, 0.15], [-1., 0.15]]))
            self._planning_horizon = p_horizon
            self._dt = 0.1
            self._env_bounds = type('', (), {})()
            self._env_bounds.y_min_map = -5     # [m]
            self._env_bounds.y_max_map = 5      # [m]
            self._env_bounds.x_min_map = -4     # [m]
            self._env_bounds.x_max_map = 4      # [m]
            self._env_bounds.y_min = -.25     # [m]
            self._env_bounds.y_max = .25      # [m]
            self._env_bounds.x_min = -.25    # [m]
            self._env_bounds.x_max = .25      # [m]
            self._goal_set = np.array(robot_goal)  # [x, y, theta]
            self._start = np.array(robot_start)              # [x, y, theta]
            self._human_goal = np.array(human_goal)
            self._human_obsv_traj = human_traj  # human observed trajectory
            # Human simulation parameters
            desired_force_factor = 2
            social_force_factor = 5.1
            obstacle_force_factor = 10
            obstacle_force_sigma = 0.1
            self._sf_params = [desired_force_factor,
                               social_force_factor,
                               obstacle_force_factor,
                               obstacle_force_sigma]
            
            # call the contoller and robot model of map2
            self._robot_model = robot_map2(self._dt, x = self._start, goal_set = self._goal_set, r = r_radius, goal_shift_radii=goal_shift_radii)
            unsafe_info = self._robot_model.unsafe_func()                # extract the unsafe set info
            map_info = self._robot_model.map_func(self._env_bounds)            # extract the environment info
            self._controller= controller_map2(unsafe_info, map_info)          # controller object

            self._vis_bound_x_min = -4
            self._vis_bound_x_max = 4
            self._vis_bound_y_min = -5
            self._vis_bound_y_max = 5
            #___________________________________________________________________
            # Environment Grid Map
            self._x_bound = 8
            self._y_bound = 10
            self._x_len = int (self._x_bound/res)
            self._y_len = int (self._y_bound/res)
            self._y_offset = 5  # to get non-negative coordinates
            self._x_offset = 4  # to get non-negative coordinates
            self._maze = np.zeros((self._y_len+2,self._x_len+2))
            self._ox, self._oy = [], []
            self._human_loc = None
            self._robot_loc = None
            self._res = res

            a = self.where([0.25,-0.25])
            b = self.where([-0.25,0.25])
            for i in range (a[1],b[1]):
             for j in range (b[0],a[0]):
                 self._maze[i][j] = 1

            # following lines add a frame to the grid map to facilitate path planning
            for i in range(0, self._x_len+2):
             self._maze[0][i] = 1

            for i in range(0, self._y_len+2):
             self._maze[i][self._x_len+2-1] = 1

            for i in range(0, self._x_len+2):
             self._maze[self._y_len+2-1][i] = 1

            for i in range(0, self._y_len+2):
             self._maze [i][0] = 1
            #_____________________________________________________________________________

        elif self._map == 'map3':
            self._name = 'map3'
            self._walls = self.extract_walls_hallway(walls_raw)
            #self.vis = Visual(walls = self._walls)
            #self.vis.render()
            self._hor_index = [0,2,4,6]
            self._ver_index = [1,3,5,7]
            

            self.human_start = human_start
            self.human_to_go = human_goal
            self.human_obs_traj = human_traj
            self.robot_start = robot_start
            self.robot_to_go =  robot_goal
            

            rl = abs(self._walls[0][0][0] - self._walls[0][1][0] ) / 2
            rw = abs(self._walls[1][0][1] - self._walls[1][1][1] ) / 2
            cl = abs(self._walls[4][0][0] - self._walls[4][1][0] ) / 2
            cw = abs (self._walls[4][0][1] )
            self._planning_horizon = p_horizon
            self._dt = 0.1
            self._env_bounds = type('', (), {})()
            self._env_bounds.y_max_map = self._walls[0][0][1]            # [m]
            self._env_bounds.y_min_map = self._walls[2][0][1]            # [m]
            self._env_bounds.x_min_map = self._walls[1][0][0]            # [m]
            self._env_bounds.x_max_map = self._walls[7][0][0]            # [m]
            self._env_bounds.y_min_park = self._walls[4][0][1]           # [m]
            self._env_bounds.x_min_park = self._walls[3][0][0]            # [m]
            self._env_bounds.x_max_park = self._walls[5][0][0]            # [m]
            self._goal_set = np.array(robot_goal)  # [x, y, theta]
            self._start = np.array(robot_start)                     # [x, y, theta]
            self._human_goal = np.array(human_goal)
            self._human_obsv_traj = human_traj  # human observed trajectory
            # Human simulation parameters
            desired_force_factor = 2.5
            social_force_factor = 4
            obstacle_force_factor = 0.5
            obstacle_force_sigma = 0.05
            self._sf_params = [desired_force_factor,
                               social_force_factor,
                               obstacle_force_factor,
                               obstacle_force_sigma]

            # call the contoller and robot model of map3
            self._robot_model = robot_map3(self._dt, x = self._start, goal_set = self._goal_set, r = r_radius, goal_shift_radii=goal_shift_radii)
            unsafe_info = self._robot_model.unsafe_func()                # extract the unsafe set info
            map_info = self._robot_model.map_func(self._env_bounds)      # extract the environment info
            self._controller= controller_map3(unsafe_info, map_info)     # controller object

            self._vis_bound_x_min = self._walls[1][0][0]
            self._vis_bound_x_max = self._walls[7][0][0]
            self._vis_bound_y_min = self._walls[4][1][1]
            self._vis_bound_y_max = self._walls[0][1][1]
            #___________________________________________________________________
            # Environment Grid Map
            self._x_bound = 2*rl
            self._y_bound = cw + rw
            self._x_len = math.ceil (self._x_bound/res)
            self._y_len = math.ceil( (self._walls[0][0][1] - self._walls[4][0][1])/res)
            self._y_offset = -self._walls[4][1][1] # to get non-negative coordinates
            self._x_offset = -self._walls[1][0][0] # to get non-negative coordinates
            self._maze = np.zeros((self._y_len+2,self._x_len+2))
            self._ox, self._oy = [], []
            self._human_loc = None
            self._robot_loc = None
            self._res = res

            a = self.where([ self._walls[3][0][0],self._walls[3][0][1]  ])
            
            for i in range (0,a[1]+1):
                for j in range (0,a[0]+1):
                    self._maze[i][j] = 1

            a = self.where(self._walls[4][1])
            b = self.where(self._walls[6][1])
            for i in range (a[1] , b[1]+1):
                for j in range (a[0],b[0]+1):
                    self._maze[i][j] = 1

            for i in range(0, self._x_len+2):
                self._maze[0][i] = 1

            for i in range(0, self._y_len+2):
                self._maze[i][self._x_len+2-1] = 1

            for i in range(0, self._x_len+2):
                self._maze[self._y_len+2-1][i] = 1

            for i in range(0, self._y_len+2):
                self._maze [i][0] = 1
            #____________________________________________________________________________
            #self.show()
        elif self._map == 'map4':
            self._name = 'map4'
            l = 10         # room length
            w = 8          # room width
            u_width = 2    # upper corridor width
            c_width = 2    # u-shape corridor width
            ow = 1.5       # right obstacle width
            self._walls.append(np.array([[0, 0],[l, 0]]))
            self._walls.append(np.array([[0, w],[l, w]]))
            self._walls.append(np.array([[0, 0],[0, w]]))
            self._walls.append(np.array([[l, 0],[l, w]]))
            self._walls.append(np.array([[l-ow, 0],[l-ow, w-u_width]]))
            self._walls.append(np.array([[l-ow, w-u_width],[l, w-u_width]]))
            self._walls.append(np.array([[c_width, c_width],[l-c_width-ow, c_width]]))
            self._walls.append(np.array([[c_width, c_width],[c_width, w-u_width]]))
            self._walls.append(np.array([[l-c_width-ow, c_width],[l-c_width-ow, w-u_width]]))
            self._planning_horizon = p_horizon
            self._dt = 0.1
            self._env_bounds = type('', (), {})()
            self._env_bounds.y_min_map = 0                      # [m]
            self._env_bounds.y_max_map = w                      # [m]
            self._env_bounds.x_min_map = 0                      # [m]
            self._env_bounds.x_max_map = l                      # [m]
            self._env_bounds.y_max = w-u_width          # [m]
            self._env_bounds.y_min = c_width            # [m]
            self._env_bounds.x_max = l-ow               # [m]
            self._env_bounds.x_mid = l-c_width-ow       # [m]
            self._env_bounds.x_min = c_width            # [m]
            self._goal_set = np.array(robot_goal)           # [x, y, theta]
            self._start = np.array(robot_start)             # [x, y, theta]
            self._human_goal = np.array(human_goal)
            self._human_obsv_traj = human_traj              # human observed trajectory
            # Human simulation parameters
            desired_force_factor = 2
            social_force_factor = 5.1
            obstacle_force_factor = 2
            obstacle_force_sigma = 0.1
            self._sf_params = [desired_force_factor,
                               social_force_factor,
                               obstacle_force_factor,
                               obstacle_force_sigma]

            # call the contoller and robot model of map3
            self._robot_model = robot_map4(self._dt, x = self._start, goal_set = self._goal_set, r = r_radius, goal_shift_radii=goal_shift_radii)
            unsafe_info = self._robot_model.unsafe_func()                # extract the unsafe set info
            map_info = self._robot_model.map_func(self._env_bounds)      # extract the environment info
            self._controller= controller_map4(unsafe_info, map_info)     # controller object

            self._vis_bound_x_min = 0
            self._vis_bound_x_max = l
            self._vis_bound_y_min = 0
            self._vis_bound_y_max = w
            #___________________________________________________________________
            # Environment Grid Map
            self._x_bound = l
            self._y_bound = w
            self._x_len = int(self._x_bound/res)
            self._y_len = int(self._y_bound/res)
            self._y_offset = 0  # to get non-negative coordinates
            self._x_offset = 0  # to get non-negative coordinates
            self._maze = np.zeros((self._y_len+2,self._x_len+2))
            self._ox, self._oy = [], []
            self._human_loc = None
            self._robot_loc = None
            self._res = res


            a = self.where([c_width,w-u_width])
            b = self.where([l-c_width-ow,c_width])
            for i in range (b[1],a[1]):
                for j in range (a[0],b[0]):
                    self._maze[i][j] = 1

            a = self.where([l-ow,w-u_width])
            b = self.where([l,0])
            for i in range (b[1] , a[1]):
                for j in range (a[0],b[0]):
                    self._maze[i][j] = 1

            for i in range(0, self._x_len+2):
                self._maze[0][i] = 1

            for i in range(0, self._y_len+2):
                self._maze[i][self._x_len+2-1] = 1

            for i in range(0, self._x_len+2):
                self._maze[self._y_len+2-1][i] = 1

            for i in range(0, self._y_len+2):
                self._maze [i][0] = 1
            #____________________________________________________________________________


    def extract_walls_hallway(self, raw_info):
        poses = raw_info.arr.poses
        points = []
        walls = []
        for p in poses:
            points.append ([p.position.x, p.position.y])
        for i in range (7):
            walls.append(np.array([points[i], points[i+1]]))
        walls.append(np.array([points[7], points[0]]))
    
        return walls
    
    def find_angle(self, ref_wall):
        start = ref_wall [0]
        end = ref_wall [1]
        dx = end[0] - start[0]
        dy = end[1] - start [1]
        self.angle = math.atan2(dy,dx) + 3.14159

    def trasnform (self, point):      
        cos_angle = math.cos(-self.angle)
        sin_angle = math.sin(-self.angle)
        transformed_x = point[0] * cos_angle - point[1] * sin_angle + self.disp 
        transformed_y = point[0] * sin_angle + point[1] * cos_angle + self.disp
        if len(point) > 2:
            transformed_yaw = point[2] + self.angle
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
            point_yaw = transformed_point[2] - self.angle
            return [point_x, point_y, point_yaw]
        else: return [point_x, point_y]
    
    def transform_path (self, input_path):
        path = []
        for i in range (len(input_path)):
            path.append (self.transform (input_path[i]))
        return path

    def transform_reverse_path (self, path_input):
        print (self.angle)
        path = []
        for i in range (len(path_input)):
            path.append(self.trasnform_reverse(path_input[i]))
        return path


    def find_displacement (self, points):
        minim = np.inf
        for p in points:
            if min(p) < minim:
                minim = min(p) 
        self.disp = 4*abs(minim)


    def is_occupied (self, location):
        dim = self._maze.shape
        loc = self.where (location)
        x = loc [0]
        y = loc [1]
        if ((not (0 <= x < dim[1])) or (not (0 <= y < dim[0]))): 
            return True
        if (self._maze[y][x] == 1): 
            return True
        else: 
            return False


    def where (self, location):
        x_loc = location[0]
        y_loc = location[1]
        y = int ((y_loc + self._y_offset)/self._res) + 1
        x = int ((x_loc + self._x_offset)/self._res) + 1
        return [x,y]

    def show (self):
        # free = 0, obstacle = 1, path = 2, start = 3, goal = 4, perseption =5
        cmap = colors.ListedColormap(['white', 'black','blue', 'green', 'red', 'gray'])
        bounds = [-0.5,0.5,1.5,2.5,3.5,4.5, 5.5]
        norm = colors.BoundaryNorm(bounds, cmap.N)

        fig, ax = plt.subplots()
        plt.figure(figsize=(16, 12), dpi=80)

        ax.imshow(self._maze, cmap=cmap, norm=norm)

        # draw gridlines
        ax.grid(which='minor', axis='both', linestyle='-', color='k', linewidth=2)

        ax.invert_yaxis()
        plt.show()


class Visual:
    def __init__ (self, walls, x_lim = [-7.1,7.1], y_lim = (-2,6.7), back_c = '#5d5d5d', mon_size = (14, 14), face_c = '#d56c11'):
        self._walls = walls
        self._xlim = x_lim
        self._ylim = y_lim
        self._back_color = back_c
        self._monitor_size = mon_size
        self._human_body_color = face_c
        self._fig = plt.figure(figsize=self._monitor_size)
        self._ax = plt.gca()
                

    def vis_walls(self, ax):
        for wall in self._walls:
            plt.plot(wall[0:2,0],wall[0:2,1], "k")

 

    
    
    def render(self, maze = []):
                #plt.subplot(1, 2, 1)
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                ax = plt.gca()
                ax.cla()
                self.vis_walls(ax)
                #plt.xlim(self._xlim[0], self._xlim[1])
                #plt.ylim(self._ylim[0], self._ylim[1])


                plt.xlabel("X [m]", fontsize=16)
                plt.ylabel("Y [m]", fontsize=16)
                fig = plt.gcf()
                fig.set_size_inches(4, 4)
                plt.show()
    
            

def close_event():
    plt.close()