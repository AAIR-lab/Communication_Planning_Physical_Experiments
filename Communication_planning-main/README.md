# Communication Planning for Cobots
This repository contains the source code for implementing the [joint communication and motion planning method](https://www.mdadvar.net/delib-comm-planning) in real-world scenarios.

## Code Structure
This repository splits the source code into two main categories:
1. **Planner Scripts**: planner scripts folder at `~/catkin_ws/src/communication_planning/planner_scripts/` contains all the Python scripts to run the main routine of the planner, i.e., `~/planner_scripts/rrt_unicycle_cbf_static_obstacle.py`. There is also a `~/planner_scripts/grid_replay.py` file among planner scripts that replays and visualizes the logged scenarios. All logged scenarios should be saved under the corresponding map folder in `~/planner_scripts/`.
Here are the short descriptions for the most important scripts in this directory:
   
      | script name | short description | 
      |-----------------|-----------------|
      | rrt_unicycle_cbf_static_obstacle.py | main routine of this method |
      | ros_helper.py | all the functions for ROS communications |
      | HighLevel.py | communication planning algorithms |
      | env.py | world representation codes |
      | gird.py | all grid-based operations, mostly accessed by HighLevel.py |
      | replay_grid.py | GUI for replaying and analyzing the logged experiments |
      | rrt.py | implementation of CBF-TB-RRT method |
      | talk.py | plays the robot's communication signal at each planning cycle |

   
3. **Robot Scripts**

## Running the Code
The planner script can be run as follows:

```bash
rosrun communication_planning rrt_unicycle_cbf_static_obstacle.py
```

The Scenario Replay GUI can be run using the following command:
   
```bash
rosrun communication_planning grid_replay.py
```


