#!/usr/bin/env python

'''
Universidade Federal de Minas Gerais (UFMG) - 2020
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
Victor R. F. Miranda, <victormrfm@gmail.com>
'''



import rospy


from math import pi, sqrt, sin, cos, tan, atan
import numpy as np

# from scipy import linalg as LA

from matplotlib.pyplot import plot, show

import copy

from geometry_msgs.msg import PointStamped, Polygon, Point32
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool





# points = [[0.0, 0.0], [1.0, 1.0], [1.0, 2.0], [0.0, 2.0]]

def planner(N_interpolation,path,r_pos,pub_marker_array):

    # N_interpolation = 10;

    points = [r_pos] + path

    n = len(points)


    M = [[0, 0, 0, 1],[1, 1, 1, 1],[0, 0, 1, 0],[3, 2, 1, 0]]

    Minv = [[2, -2, 1, 1],[-3, 3, -2, -1],[0, 0, 1, 0],[1, 0, 0, 0]]

    T = []

    for k in range(n):
    	if k == 0:
    		v = [points[k+1][0]-points[k][0], points[k+1][1]-points[k][1]]
    	elif k==(n-1):
    		v = [points[k][0]-points[k-1][0], points[k][1]-points[k-1][1]]
    	else:
    		v = [(points[k+1][0]-points[k-1][0])/2.0, (points[k+1][1]-points[k-1][1])/2.0]

    	T.append(v)




    s_vec = [i/float(N_interpolation) for i in range(N_interpolation)]
    path = [[],[]]


    for k in range(n-1):


    	A = [[points[k][0],points[k][1]], [points[k+1][0],points[k+1][1]], [T[k][0],T[k][1]], [T[k+1][0],T[k+1][1]]]

    	# print np.matrix(A)
    	# print ""
    	ck = np.matrix(Minv)*np.matrix(A)
    	

    	for s in s_vec:

    		
    		path[0].append(ck[0,0]*s**3+ck[1,0]*s**2+ck[2,0]*s**1+ck[3,0]*s**0)
    		path[1].append(ck[0,1]*s**3+ck[1,1]*s**2+ck[2,1]*s**1+ck[3,1]*s**0)


    path[0].append(ck[0,0]+ck[1,0]+ck[2,0]+ck[3,0])
    path[1].append(ck[0,1]+ck[1,1]+ck[2,1]+ck[3,1])

    print("[X_p,Y_p] = [%f, %f]" % (path[0][-1],path[1][-1]))

    a = np.matrix(points).transpose().tolist()

    send_marker_array_to_rviz(path, pub_marker_array)

    #plot(a[0],a[1],'ro')
    #plot(path[0],path[1],'k.-')
    #show()

    return path


def send_marker_array_to_rviz(traj, pub_marker_array):
    """Function to send a array of markers, representing the curve, to rviz
    :param traj: trajectory list of points
    :param pub_rviz: ROS publisher object
    :return:
    """
    if not pub_marker_array:
        raise AssertionError("pub_marker_array is not valid:%s".format(pub_marker_array))

    points_marker = MarkerArray()
    for i in range(len(traj[0])):
        marker = Marker()
        marker.header.frame_id = "/os1_init"
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        # Duration
        # marker.lifetime = rospy.Duration.from_sec(1.0)
        # Size of sphere
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        # Color and transparency
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        # Pose
        marker.pose.orientation.w = 1.0

        px = traj[0][i]
        py = traj[1][i]
        # px, py = traj[i]

        marker.pose.position.x = px
        marker.pose.position.y = py
        marker.pose.position.z = 0.1

        # Append marker to array
        points_marker.markers.append(marker)

    # Publish marker array
    pub_marker_array.publish(points_marker)#




def callback_start(msg):
    global path, points, pub_traj, robot_position, pub_marker
    N = 5
    if(msg.data == True):
        path = planner(N, points, robot_position,pub_marker)

        poly_msg = Polygon()
        

        for i in range(len(path[0])):
            point_msg = Point32()
            point_msg.x = path[0][i]
            point_msg.y = path[1][i]
            point_msg.z = 0.0

            poly_msg.points.append(point_msg)



        pub_traj.publish(poly_msg)


def callback_odom(msg):
    global robot_position
    robot_position = [msg.pose.pose.position.x,msg.pose.pose.position.y]


def callback_new_point(msg):
    global points
    points.append([msg.point.x,msg.point.y])
    print("%d points" % len(points))


def callback_clear(msg):
    global points
    if (msg.data == True):
        points = []
        print("points cleaned")


# Rotina primaria
def run():
    global freq
    global pub_rviz_ref, pub_rviz_pose
    global points, robot_position
    global pub_traj, pub_marker

    points = []


    rospy.init_node("trajectory_planner_example_traj")

    pub_traj = rospy.Publisher("/ref_path", Polygon, queue_size=10)
    pub_marker = rospy.Publisher("/vizual_path", MarkerArray, queue_size=10)


    rospy.Subscriber("clicked_point", PointStamped, callback_new_point)
    rospy.Subscriber("/ekf_odom", Odometry, callback_odom)
    rospy.Subscriber("/start_planner", Bool, callback_start)
    rospy.Subscriber("/clear_planner", Bool, callback_clear)


    # Wait a bit
    rate = rospy.Rate(freq)



    while not rospy.is_shutdown():
     # Wait a bit
        rate.sleep()
# ---------- !! ---------- !! ---------- !! ---------- !! ----------








# Main function
if __name__ == '__main__':

    # Frequency of the loop
    global freq
    freq = 10.0  # Hz


    try:
        run()
    except rospy.ROSInterruptException:
        pass

    rospy.sleep(.5)
