# Espeleorobo planning

This repository contains codes for generating and/or publishing reference paths for the espeleorobo


## Scripts included on this package:
- `example_trajectories.py`: Generates a example path to be followed by the robot
- `dijkstra_trajectories.py`: Reads from a txt file a trajectory generated by a Dijkstra Algorithm. The trajectories are stored on the folder txt_trajectories

## Txt format:
The trajectories written on the files `trraj_N.txt` (inside txt_trajectories) must have the number of sampled pioints on the first line. Each of the following lines corresponds to the coordinates `x`, `y` and `z` separated by a `\t`.


## How to interact - vec_field_control.py

This code implements a vetor field method to cenerage input commands to the robot so that it converges to a desired path

**Topics:**
- `/espeleo/traj_points`  (message type:`geometry_msgs/Polygon`): Subscribe to this topic to get a sequence of points representing a path

**Input parameters:**

There is a file `planning_params.yaml` that contain some parameters for this package. They are listed below.

Parameters for the example trajectories:
- `N_curve`: number of example curve to be generated, from 1 to 3
- `N_points`: number of points of the curve to be sampled, 200 for example
- `a`: stretch factor of the curve in the x direction
- `b`: stretch factor of the curve in the y direction
- `phi`: rotation of the curve around the z axis (in dregrees)
- `cx`: displacement in the x direction (meters)
- `cy`: displacement in the y direction (meters)

Parameters for choosing the Dijkstra trajectory:
- `dijkstra_traj_number`: Number of the Dijkstra trajectory


**Usage:**

With roscore and vrep running (in this order), run the following launch flie (remember to select you scene):

`roslaunch espeleo_vrep_simulation basic.launch`

Now, the robot is already waiting for a trajectory to be followed. An example trajectory can be generated with the following command:

`roslaunch espeleo_planning launch_trajectory.launch`


In order to select an example trajectory or a trajectory generated by the Dijkstra the user must edit the launch file `launch_trajectory.launch`. One of the options is commented while the other is not
