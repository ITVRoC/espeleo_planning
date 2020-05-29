#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Polygon, Point
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
from tf2_msgs.msg import TFMessage
import numpy as np
import sys


"""
Universidade Federal de Minas Gerais (UFMG) - 2019
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
"""


# Function to generate an ellipse path
def refference_trajectory_1(N):

    global a, b, phi, cx, cy

    # Geometric parameters
    # a = 3 # semiaxis x
    # b = 1.5 # semiaxis y
    # cx = 0 # center x
    # cy = 0 # center y
    # phi = pi / 4 # rotation angle of the curve

    # Parameter
    dp = 2*pi/N
    p = -dp

    # Loop to sample the curve
    traj = [[],[]]
    for k in range(N):

        # Increment parameter
        p = p + dp

        # Compute a point of the ellipse in a local frame
        x_ref0 = a * cos(p)
        y_ref0 = b * sin(p)

        # Rotate and displace the point
        x_ref = cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx * 1
        y_ref = sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy * 1

        # Save the computed point
        traj[0].append(x_ref)
        traj[1].append(y_ref)

    return (traj)

# ----------  ----------  ----------  ----------  ----------




# Function to generate an "8 like" path
def refference_trajectory_2(N):

    global a, b, phi, cx, cy

    # Geometric parameters
    # a = 3.0 # height of the "8"
    # b = 1.5 # width of the "8"
    # cx = 0 # center x
    # cy = 0 # center y
    # phi = pi / 4 # rotation angle of the curve

    # Parameter
    dp = 2*pi/N
    p = -dp

    # Loop to sample the curve
    traj = [[],[]]
    for k in range(N):

        # Increment parameter
        p = p + dp

        # Compute a point of the "8" in a local frame
        x_ref0 = a * sin(p)
        y_ref0 = b * sin(2.0*p)

        # Rotate and displace the point
        x_ref = cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx * 1
        y_ref = sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy * 1

        # Save the computed point
        traj[0].append(x_ref)
        traj[1].append(y_ref)



    return (traj)
# ----------  ----------  ----------  ----------  ----------





# Function to generate a "rectangular" path
def refference_trajectory_3(N):

    global a, b, phi, cx, cy

    # Geometric parameters
    # a = 2**(-4) #
    # b = 0 #
    # c = 2**(-4) #
    # cx = 0 # center x
    # cy = 0 # cewnter y
    # phi = 0 # rotation angle of the curve

    # Parameter
    dp = 2*pi/N
    p = -dp

    traj = [[],[]]
    for k in range(N):

        # Increment parameter
        p = p + dp

        # Compute a point of the "rectangular" in a local frame
        r = (1.0*cos(p)**4 + 0.0*cos(p)**2*sin(p)**2 + 1.0*sin(p)**4)**(-0.25)
        x_ref0 = a * r * cos(p)
        y_ref0 = b * r * sin(p)

        # Rotate and displace the point
        x_ref = cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx * 1
        y_ref = sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy * 1

        # Save the computed point
        traj[0].append(x_ref)
        traj[1].append(y_ref)

    return (traj)
# ----------  ----------  ----------  ----------  ----------




# Function to generate a "sinoidal" path - That is an open path
def refference_trajectory_4(N):

    global a, b, phi, cx, cy

    # Geometric parameters
    # a = 2**(-4) #
    # b = 0 #
    # c = 2**(-4) #
    # cx = 0 # center x
    # cy = 0 # cewnter y
    # phi = 0 # rotation angle of the curve

    # Parameter
    dp = 2*pi/N
    p = -dp

    traj = [[],[]]
    for k in range(N):

        # Increment parameter
        p = p + dp

        # Compute a point of the "rectangular" in a local frame
        x_ref0 = p-pi
        y_ref0 = sin(x_ref0)

        # Rotate and displace the point
        x_ref = cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx * 1
        y_ref = sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy * 1

        # Save the computed point
        traj[0].append(x_ref)
        traj[1].append(y_ref)

    return (traj)
# ----------  ----------  ----------  ----------  ----------



""" # CREATE HERE A NEW CURVE
# Function to generate a "??" path
def refference_trajectory_5(N):

    # Geometric parameters
    # a = ... # PLACE HERE YOUR PARAMETER
    # b = ... # PLACE HERE YOUR PARAMETER
    # c = ... # PLACE HERE YOUR PARAMETER
    cx = 0 # center x
    cy = 0 # cewnter y
    phi = 0 # rotation angle of the curve

    # Parameter
    dp = 2*pi/N
    p = -dp

    traj = [[],[]]
    for k in range(N):

        # Increment parameter
        p = p + dp

        # Compute a point of the "rectangular" in a local frame
        # x_ref0 = ... # PLACE HERE YOUR CURVE EQUATION
        # y_ref0 = ... # PLACE HERE YOUR CURVE EQUATION

        # Rotate and displace the point
        x_ref = cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx * 1
        y_ref = sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy * 1

        # Save the computed point
        traj[0].append(x_ref)
        traj[1].append(y_ref)

    return (traj)
# ----------  ----------  ----------  ----------  ----------
"""


# Function to create a message of the type polygon, which will carry the points of the curve
def create_traj_msg(traj):

    # Create 'Polygon' message (array of messages of type 'Point')
    traj_msg = Polygon()
    p = Point()
    for k in range(len(traj[0])):
        # Create point
        p = Point()
        # Atribute values
        p.x = traj[0][k]
        p.y = traj[1][k]
        p.z = 0.0
        # Append point to polygon
        traj_msg.points.append(p)

    return traj_msg
# ----------  ----------  ----------  ----------  ----------



# Function to send a array of markers, representing the curve, to rviz
def send_curve_to_rviz(traj,pub_rviz):

    # Create messsage
    points_marker = MarkerArray()
    marker = Marker()
    # Iterate over the points
    for k in range(len(traj[0])):
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()
        marker.id = k
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        # Size of sphere
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        # Color and transparency
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        # Pose
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = traj[0][k]
        marker.pose.position.y = traj[1][k]
        marker.pose.position.z = 0.1
        # Append marker to array
        points_marker.markers.append(marker)

    pub_rviz.publish(points_marker)

    return (points_marker)
# ----------  ----------  ----------  ----------  ----------






# Rotina primaria
def trajectory():
    global freq
    global pub_rviz_ref, pub_rviz_pose


    pub_traj = rospy.Publisher("/espeleo/traj_points", Polygon, queue_size=10)
    pub_rviz_curve = rospy.Publisher("/visualization_trajectory", MarkerArray, queue_size=1)
    rospy.init_node("trajectory_planner")

    # Wait a bit
    rate = rospy.Rate(freq)


    # Generate one of the curve types
    if curve_number == 1:
        traj = refference_trajectory_1(number_of_samples)
    elif curve_number == 2:
        traj = refference_trajectory_2(number_of_samples)
    elif curve_number == 3:
        traj = refference_trajectory_3(number_of_samples)
    elif curve_number == 4:
        traj = refference_trajectory_4(number_of_samples)
    # PLACE HERE A NEW FUNCTION
    # elif curve_number == 5:
    #     traj = refference_trajectory_5(number_of_samples)
    else:
        print "Invalid curve_number !"

    # Create message with the points of the curve
    traj_msg = create_traj_msg(traj)

    # Wait a bit
    rate = rospy.Rate(freq)

    # Publish the message
    pub_traj.publish(traj_msg)

    print "\33[99m----------------------------\33[99m"
    print "\33[99mCurve created and publhished\33[99m"
    print "\33[99mCurve type: ", curve_number, "\33[99m"
    print "\33[99mSampled samples: ", number_of_samples, "\33[99m"
    print "\33[99m----------------------------\33[99m"

    # Send curve to rviz
    sleep(1.0)
    send_curve_to_rviz(traj, pub_rviz_curve)



    while not rospy.is_shutdown():

        rate.sleep()

        break

# ---------- !! ---------- !! ---------- !! ---------- !! ----------






# Main function
if __name__ == '__main__':

    # Frequency of the loop
    global freq
    freq = 10.0  # Hz

    # Input parameters
    global pkg_path, curve_number, number_of_samples, a, b, phi, cx, cy
    
    # Obtain the parameters
    try:
        pkg_path = rospy.get_param("/trajectory_planner/pkg_path");
        curve_number = int(rospy.get_param("/trajectory_planner/N_curve"));
        number_of_samples = int(rospy.get_param("/trajectory_planner/N_points"));
        a = float(rospy.get_param("/trajectory_planner/a"));
        b = float(rospy.get_param("/trajectory_planner/b"));
        phi = float(rospy.get_param("/trajectory_planner/phi"))*(3.1415926535/180.0);
        cx = float(rospy.get_param("/trajectory_planner/cx"));
        cy = float(rospy.get_param("/trajectory_planner/cy"));
    except:
        print "\33[41mProblem occurred when trying to read the parameters!: example_trajectories.py\33[0m"



    try:
        trajectory()
    except rospy.ROSInterruptException:
        pass
