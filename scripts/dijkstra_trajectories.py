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
# from scipy.spatial.transform import Rotation
import numpy as np
import sys


"""
Universidade Federal de Minas Gerais (UFMG) - 2019
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
"""





# Function to read a reference curve from a txt file
def read_txt_trajectory(id):

    path = pkg_path+"/txt_trajectories/traj_"+str(id)+".txt"

    traj = [[],[],[]]

    file_obj = open(path, 'r')

    i = 1
    with open(path) as f:
        for line in f:  # Line is a string
            numbers_str = line.split()
            numbers_float = [float(x) for x in numbers_str]  # convert to float
            if (i == 1):
                n = int(numbers_float[0]) # read number of samples
            else:
                traj[0].append(numbers_float[0]) #read x coordinate of point
                traj[1].append(numbers_float[1]) #read y coordinate of point
                traj[2].append(numbers_float[2]) #read z coordinate of point
            i = i + 1

    file_obj.close()

    #print 'traj = \n', traj

    return (traj, n)
# ----------  ----------  ----------  ----------  ----------





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
        p.z = traj[2][k]
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
        marker.pose.position.z = traj[2][k]
        # Append marker to array
        points_marker.markers.append(marker)

    pub_rviz.publish(points_marker)

    return (points_marker)
# ----------  ----------  ----------  ----------  ----------






# Rotina primaria
def trajectory():
    global freq
    global pub_rviz_ref, pub_rviz_pose


    pub_traj = rospy.Publisher("/espeleo/traj_points", Polygon, queue_size=1)
    pub_rviz_curve = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)
    rospy.init_node("trajectory_planner")

    # Wait a bit
    rate = rospy.Rate(freq)


    # Generate one of the curve types
    if curve_number > 0 and curve_number <=4:
        traj, number_of_samples = read_txt_trajectory(curve_number)
    else:
        print "Invalid curve_number !"

    # Create message with the points of the curve
    traj_msg = create_traj_msg(traj)

    # Wait a bit
    rate = rospy.Rate(freq)

    # Publish the message
    pub_traj.publish(traj_msg)

    print "----------------------------"
    print "Curve read and publhished"
    print "Curve type: ", curve_number
    print "Sampled samples: ", number_of_samples
    print "----------------------------"

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
    global pkg_path, curve_number, number_of_samples
    # Obtain the parameters
    # curve_number = int(sys.argv[1])
    try:
        pkg_path = rospy.get_param("/trajectory_planner/pkg_path");
        curve_number = int(rospy.get_param("/trajectory_planner/dijkstra_traj_number"));
    except:
        print "\33[41mProblem occurred when trying to read the parameters!: dijkstra_trajectories.py\33[0m"


    try:
        trajectory()
    except rospy.ROSInterruptException:
        pass
