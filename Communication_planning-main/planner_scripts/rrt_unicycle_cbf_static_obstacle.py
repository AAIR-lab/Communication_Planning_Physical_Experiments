#!/usr/bin/env python

"""

CBF-BELIEF-based RRT

"""
import matplotlib.pyplot as plt
from human import Human
from rrt import RRT
from simulator import human_motion_model, set_config_params
import numpy as np
from env import ENV
from HighLevel import HighPlanner, Visual
from measure import Measurements
import ros_helper
import rospy


show_animation = True
real_time_report = True
to_go_path = []
test_weights =[
    [0.25,1.5],
    [0.5, 1.5],
    [0.75,1.5],
    [1,1.5],
    [1.25, 1.5],
    [1.5, 1.5],
    [1.5, 1.25],
    [1.5, 1],
    [1.5, 0.75],
    [1.5, 0.5],
    [1.5, 0.25],
    [1.5, 0.01]
    ]
w_ratio = []
for w in test_weights:
    w_ratio.append(w[0]/w[1])
w_index = 10
#___________________________________Hyper Parameters____________________________________________________
high_level_mode = 'on'                # 'on' or 'off'
experiment_mode = 'table'
# global parameters
human_radius = 0.2
robot_radius = 0.3
grid_res = 0.3
env_map = 'map3'

priority = 'H'
if priority == 'R': cost_weights = [1.5, 0.25, 4, 0.1, 1]
else: cost_weights = [0.25, 1.5, 1.5, 0.1, 0]
h_pers_size = 5
  # weights = [robot_cost_to goal, human_cost_to_goal, path_conflict_cost, communication cost, heading cost]
h_avg_vel = 1                       # human average velocity

if experiment_mode == 'weights': 
    dir = 'weights' + '\\' + env_map
else: 
    dir = env_map
    file_name = env_map + '_' + high_level_mode + '_' + priority + '_'
if experiment_mode == 'weights':
    cost_weights = [test_weights[w_index][0], test_weights[w_index][1], 2, 0.1, 0]

## rrt parameters
rrt_goal_shift_radii = 2
rrt_state_sample_var = 1
if high_level_mode == 'on':
    rrt_max_iter = 50
    rrt_num_segments = 5
    rrt_state_sample_mode = 'explore_oriented'   # 'exploit_oriented' and 'explore_oriented'
    planning_horizon = 2
    rrt_cost_weights = [1, 1.5, 1, 1]     # [w_goal, w_human, w_heading, w_trap]
else:
    rrt_max_iter = 50
    rrt_num_segments = 5
    rrt_state_sample_mode = 'explore_oriented'
    planning_horizon = 1.5
    rrt_cost_weights = [1, 1.5, 1, 1]     # [w_goal, w_human, w_heading, w_trap]



#___________________________________________________________________________________________________________

def main():
    rospy.Duration(160)
    r_goal, h_goal = ros_helper.get_agents_goals()
    r_start, h_start, human_visible = ros_helper.get_agents_location()
    #if h_start == [0.0,0.0]: h_start = [11.5, 9.5]
    h_obsrv_traj = np.array([h_start, h_start])
    world = ENV (map_name = env_map, 
            res = grid_res, 
            p_horizon = planning_horizon, 
            human_start = h_start, 
            human_goal = h_goal, 
            human_traj = h_obsrv_traj, 
            robot_start =  r_start, 
            robot_goal = r_goal, 
            r_radius = robot_radius,
            goal_shift_radii = rrt_goal_shift_radii)
    walls = world._walls

    h_start = world.human_start 
    h_goal = world.human_to_go 
    h_obsrv_traj = world.human_obs_traj 
    r_start = world.robot_start 
    r_goal = world.robot_to_go
    
    print ("robot loc")
    print (r_start)
    print ("human loc")
    print (h_start)
    print ("robot to go")
    print (r_goal)
    print ("human to go")
    print (h_goal)
    
    planner_publisher = ros_helper.create_publisher()


    env_bounds = world._env_bounds
    print("start " + __file__)
    # planning parameters
    dt = world._dt                                          # sample time [s] 
    measure = Measurements(human_radius = human_radius, r_goal = r_goal [-1][0:2], h_goal = h_goal)
    # Enviroment Bounds
    env_bounds = world._env_bounds
    # robot parameters
    goal_set = world._goal_set                              # [x, y, theta]
    start = world._start                                    # [x, y, theta]
    robot = world._robot_model
    rrt = RRT(env_bounds = env_bounds,
                robot_model = robot, 
                controller_model = world._controller, 
                planning_horizon = planning_horizon,
                n_segments = rrt_num_segments,
                max_iter = rrt_max_iter,
                cost_weights = rrt_cost_weights,
                state_sample_var = rrt_state_sample_var,
                state_sample_mode = rrt_state_sample_mode,
                occupancy_grid = world._maze, 
                grid_res = world._res, 
                multiple_plan = high_level_mode,
                grid_y_offset = world._y_offset,
                grid_x_offset = world._x_offset)              # initialize an rrt object

    # human parameters
    human_goal = world._human_goal
    human_obsv_traj = world._human_obsv_traj                # human observed trajectory
    human_obs_belief = []
    human = Human(dt = dt, 
                    ob_belief = human_obs_belief, 
                    predict_horizon = planning_horizon, 
                    obs_trajectory = human_obsv_traj, 
                    goal = human_goal, 
                    world = world, 
                    h_radius = human_radius)  #initialize a human object
    
    # high-level planner parameters
    # weights = [robot_cost_to goal, human_cost_to_goal, path_conflict_cost, communication cost]
    hp = HighPlanner(r_radius= robot_radius, 
                        h_goal = human_goal, 
                        weights = cost_weights, 
                        environment = world, 
                        pers_size = h_pers_size, 
                        avg_velocity = h_avg_vel, 
                        robot_goal = r_goal, 
                        h_radius = human_radius,
                        rrt_ph = planning_horizon)
    
    hp.update_logistics (r_goal = goal_set, e_walls = walls, r_start = np.copy(start))
    optimal_distances = hp.give_distance_norms (r_start[0:2], h_start)
    y_min, y_max = world._vis_bound_y_min -2, world._vis_bound_y_max + 2
    x_min, x_max = world._vis_bound_x_min -2, world._vis_bound_x_max + 2
    

    graphic = Visual(walls= walls,
                        x_lim = [x_min, x_max], 
                        y_lim = [y_min, y_max], 
                        human = human, 
                        robot = robot, 
                        rrt = rrt,
                        animation_mode = show_animation)


    # Transform walls into:
    # List of linear obstacles given in the form (x_min, x_max, y_min, y_max)
    obstacles = []
    for wall in walls:
        x_min, y_min = wall[0]
        x_max, y_max = wall[1]
        obstacles.append([x_min, x_max, y_min, y_max])

    human_waypoints = []
    human_vis_flag_waypoints = [] 
    robot_waypoints = []
    planning_cycles = 0
    # iterative motion planning
    while not robot.goal_reached_bool:
        planning_cycles = planning_cycles + 1     
        ##############
        #Tree Expansion
        best_nodes = rrt.planning()
        rrt._node_list_log.append(rrt._node_list)
        library = []
        for node in best_nodes:
            _, node_path = rrt.generate_final_course(node)
            library.append([node.x[0], node.x[1], node.cost_heading, node_path]) 
        
        ##############
        # high-level planner
        # input: 1. selected nodes, 2. tree, 
        if high_level_mode  == 'on':
            #human_obj_copy = copy.deepcopy(human)
            #human_obj_copy.dt = 0.1 #<<<<<<<<<<<<<<<--<<<< check later
            hp_out = hp.give_plan(library, robot.x[0:2], human.human_loc, human.human_dir*180/np.pi, human_visible) # [select_node_loc, x_curr_robot, list of belief based obstacles, select_node_indx]
            
            # need to show after render <<<<--<<<<
            communication_action = hp_out[1]
            human.ob_belief = hp_out[2]
            selected_node_indx = hp_out[3]
            if real_time_report:
                print('-----------------------------')
                print('high-level report:')
                print ("Planning Cycle:  " + str (planning_cycles))
                if human_visible: print ("Human was visible")
                else: print("Human was not visible")
                print("nominated nodes + costs:")
                for node in best_nodes:
                    print("node:" + str((node.x[0], node.x[1], node.x[2])) + ", cost:" + str(node.cost)+ ", cost_heading:" + str(node.cost_heading))
                print("selected node: " + str(hp_out[0]))
                print("communication action " + communication_action)
                print("belief " + str(human.ob_belief))
                print('-----------------------------')
        else:
            selected_node_indx = 0
            communication_action = ''
        ros_helper.communicate_with_human(communication_action)
        #graphic.render("", planning=True)
        ##############
        # move method to move robot and human both
        selected_node = best_nodes[selected_node_indx]
        control, path = rrt.generate_final_course(selected_node)
        path_reverse = world.transform_reverse_path(path)
        print (path)
        print (path_reverse)

        r_start, h_start, human_visible = ros_helper.get_agents_location()

        ros_helper.send_waypoints_to_robot(planner_publisher, path_reverse)
        r_way, h_way, h_vis, robot_new_loc = ros_helper.wait_for_move_base(r_start, h_start, human_visible)
        r_way[0][2] = communication_action

        human_new_loc, human_visible = h_way[-1], h_vis[-1]
        robot_new_loc = np.array (robot_new_loc)
        human_new_loc = np.array(human_new_loc)

        robot.update_loc (robot_new_loc)
        human.human_update(human_new_loc)
        robot_waypoints.append(r_way)
        human_waypoints.append(h_way)
        human_vis_flag_waypoints.append(h_vis)
        plt.close()

    wall_info = [walls, [x_min, x_max], [y_min, y_max]]
    waypoints = [robot_waypoints,human_waypoints, human_vis_flag_waypoints]
    goals = [r_goal, h_goal]
    if high_level_mode == 'on': grid_log = hp._grid_log
    else: grid_log = [[]]
    measure.log_grid_scenario(grid_log, file_name, wall_info, waypoints, planning_cycles, 
                                dir, rrt._node_list_log, optimal_distances, goals)

if __name__ == '__main__':
    main()

