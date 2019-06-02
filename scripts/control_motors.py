#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
import sys
from ros_eposmcd_msgs.msg import Movement, MovementArray, EspeleoJoints
from tf2_msgs.msg import TFMessage
import numpy as np


"""
Universidade Federal de Minas Gerais (UFMG) - 2019
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
"""



# Callback to get the pose of the robot
def callback_pose(data):
    global pos, rpy

    for T in data.transforms:
        # Chose the transform of the EspeleoRobo
        if (T.child_frame_id == "EspeleoRobo"):

            # Get the orientation
            x_q = T.transform.rotation.x
            y_q = T.transform.rotation.y
            z_q = T.transform.rotation.z
            w_q = T.transform.rotation.w
            euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
            theta_n = euler[2]

            # Get the position
            pos[0] = T.transform.translation.x
            pos[1] = T.transform.translation.y
            pos[2] = T.transform.translation.z
            rpy = euler

    return
# ----------  ----------  ----------  ----------  ----------





# Callback to get the cmv_vel for the espeleorobo
def callback_cmd_vel(data):

    global v, omega
    global last_cmd_vel_msg

    # Get the command velocities
    v = data.linear.x
    omega = data.angular.z

    # Update the "last command received" variable
    last_cmd_vel_msg = rospy.get_rostime().to_sec()

    return
# ----------  ----------  ----------  ----------  ----------




# Callback to get the positions of the wheels
def callback_joints(data):

    global jointsPositions

    # Get the wheels/legs positions
    jointsPositions = data.jointsPositions

    return
# ----------  ----------  ----------  ----------  ----------






# Rotina feedback linearization
def compute_velocities(v, omega, joints):

    global vel_vec

    # If no cmd_vel is received -> decrease the robot speed
    if (rospy.get_rostime().to_sec() - last_cmd_vel_msg >= TIME_OUT):
        # Consider a null cmd_vel
        v = 0
        omega = 0
        # Filter the command  so that the wheel speds goes to zero exponentially
        vel_vec = [0.85*vel_vec[k] for k in range(6)]

    else:
        # Velocity for the right wheels
        VD = 1*v + (L/2.0)*omega
        VD = VD/R
        # Velocity for the left wheels
        VE = 1*v - (L/2.0)*omega
        VE = VE/R

        # Contruct the wheels velocity vector
        vel_vec = [-VD, -VD, -VD, VE, VE, VE]


    # If the middle legs must be up, controll them do do so
    if SMART_LEGS:
        vel_vec[1] = 5*sin(pi-joints[1]) # Law for the right middle leg
        vel_vec[4] = 5*sin(pi-joints[4]) # Law for the left middle leg
        #vel_vec[1] = pi-joints[1] # Law for the right middle leg
        #vel_vec[4] = pi-joints[4] # Law for the left middle leg


    return vel_vec

# ----------  ----------  ----------  ----------  ----------




# Rotina primaria
def control_6_wheels():

    # Publisher for wheels velocities
    pub_velocity_cmds = rospy.Publisher("/ros_eposmcd/velocity_movement", MovementArray, queue_size=1) #declaracao do topico para comando de velocidade

    # Init node
    rospy.init_node("espeleo_control") #inicializa o no "este no"

    # Subscriber for a command velocity
    rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel) #declaracao do topico onde sera lido o estado do robo
    # Subscriber for a wheel positios
    rospy.Subscriber("/ros_eposmcd/joints_positions", EspeleoJoints, callback_joints) #declaracao do topico onde sera lido o estado do robo

    #Define uma variavel que controlar[a a frequencia de execucao deste no
    rate = rospy.Rate(freq)


    # O programa do no consiste no codigo dentro deste while
    while not rospy.is_shutdown(): #"Enquanto o programa nao ser assassinado"

        # Compute velocity of the wheels given a desired comand of velocity (v, omega)
        vel_vec = compute_velocities(v, omega, jointsPositions)


        # Create a message with the wheels velocities
        vel_6_cmd = MovementArray()
        for k in range(6):
            vel_cmd = Movement()
            vel_cmd.nodeID = k+1
            vel_cmd.velocity = vel_vec[k]
            vel_6_cmd.movement_command.append(vel_cmd)

        # Publish the message
        pub_velocity_cmds.publish(vel_6_cmd)

        #Espera por um tempo de forma a manter a frequencia desejada
        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------






# Funcao inicial
if __name__ == '__main__':

    # Frequency of the loop
    global freq
    freq = 20.0  # Hz

    global SMART_LEGS
    SMART_LEGS = True #Control the middle legs independntly

    global L # Distance between the center wheels
    L = 0.4347
    global R # Radius of the wheel
    R = 0.1997

    # Forward velocity and angular velocity
    global v, omega
    v = 0
    omega = 0

    # Position of the joints (wheels)
    global jointsPositions
    jointsPositions = [0, 0, 0, 0, 0, 0]

    # Variables to stop the robot if not cmd vel is received after a TIME_OUT
    global last_cmd_vel_msg
    last_cmd_vel_msg = 0
    global TIME_OUT
    TIME_OUT = 0.5

    # Velocities to be sent to the wheels
    global vel_vec
    vel_vec = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    try:
        control_6_wheels()
    except rospy.ROSInterruptException:
        pass
