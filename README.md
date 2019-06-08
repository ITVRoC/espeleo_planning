# Espeleorobo planning

This repository contains codes for the high level control of the espeleorobo in V-REP


## Scripts included on this package:
- example_trajectories.py: Generates a example path to be followed by the robot



## How to interact - vec_field_control.py

This code implements a vetor field method to cenerage input commands to the robot so that it converges to a desired path

**Topics:**
- `/cmd_vel`  (message type:`geometry_msgs/Twist`): Publish a velocity command (forward velocity and an angular velocity)
- `/tf`  (message type:`tf2_msgs/TFMessage`): Subscribe to this topic to get the pose of the robot. Temporary ground truth.
- `/espeleo/traj_points`  (message type:`geometry_msgs/Polygon`): Subscribe to this topic to get a sequence of points representing a path

**Input parameters:**

- `first parameter`:  number of example curve to be generated, from 1 to 3
- `second parameter`:  number of points of the curve to be sampled, 200 for example


**Usage:**

With roscore and vrep running (in this order), run the following launch flie:

`roslaunch espeleo_vrep_simulation basic.launch`

Now, the robot is already waiting for a trajectory to be followed. An example trajectory can be generated with the following command:

`rosrun espeleo_planning example_trajectories.py 1 200`

It will then generate the example curve number 1 with 200 sampled points. The robot must start to follow the curve.
