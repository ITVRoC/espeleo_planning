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



#Function to obtain a polynomial interpolation
def planner(N_interpolation,points0,r_pos,pub_marker_array):

    #Include the current robot's position in the planning
    points = [r_pos] + points0

    n = len(points)

    #Matrix associated to constrainned 3rd order polynomials
    M = [[0, 0, 0, 1],[1, 1, 1, 1],[0, 0, 1, 0],[3, 2, 1, 0]]
    Minv = [[2, -2, 1, 1],[-3, 3, -2, -1],[0, 0, 1, 0],[1, 0, 0, 0]]

    #Heurietics to define the 
    T = []
    for k in range(n):
    	if k == 0:
    		v = [points[k+1][0]-points[k][0], points[k+1][1]-points[k][1], points[k+1][2]-points[k][2]]
    	elif k==(n-1):
    		v = [points[k][0]-points[k-1][0], points[k][1]-points[k-1][1], points[k][1]-points[k-1][2]]
    	else:
    		v = [(points[k+1][0]-points[k-1][0])/2.0, (points[k+1][1]-points[k-1][1])/2.0, (points[k+1][1]-points[k-1][2])/2.0]
    	T.append(v)

    #Definition of a vector to sample some points of the computed polynomials
    s_vec = [i/float(N_interpolation) for i in range(N_interpolation)]
    path = [[],[],[]]

    #Iterate over each pair of points
    for k in range(n-1):

        #Write the matrix that contains the boundary conditions for the polynomials
        A = [[points[k][0],points[k][1]], [points[k+1][0],points[k+1][1]], [T[k][0],T[k][1]], [T[k+1][0],T[k+1][1]]]
        #A = [[points[k][0],points[k][1],points[k][2]], [points[k+1][0],points[k+1][1],points[k+1][2]], [T[k][0],T[k][1],T[k][2]], [T[k+1][0],T[k+1][1],T[k+1][2]]]

        #Compute the coefficients
    	ck = np.matrix(Minv)*np.matrix(A)
    	
        #Sample the computed polynomials
    	for s in s_vec:
            path[0].append(ck[0,0]*s**3+ck[1,0]*s**2+ck[2,0]*s**1+ck[3,0]*s**0)
            path[1].append(ck[0,1]*s**3+ck[1,1]*s**2+ck[2,1]*s**1+ck[3,1]*s**0)
            path[2].append(points[k][2]*(1.0-s)+points[k+1][2]*(s))
            #path[2].append(ck[0,2]*s**3+ck[1,2]*s**2+ck[2,2]*s**1+ck[3,2]*s**0)

    #Include the last point on the path
    path[0].append(ck[0,0]+ck[1,0]+ck[2,0]+ck[3,0])
    path[1].append(ck[0,1]+ck[1,1]+ck[2,1]+ck[3,1])
    path[2].append(points[k+1][2])
    #path[2].append(ck[0,2]+ck[1,2]+ck[2,2]+ck[3,2])

    #Publish the computed path to be visualized on rviz
    send_marker_array_to_rviz(path, pub_marker_array)

    # #Plot the computed path
    # plot(a[0],a[1],'ro')
    # plot(path[0],path[1],'k.-')
    # show()

    return path



#Function to send a marker array to rviz
def send_marker_array_to_rviz(traj, pub_marker_array):
    """Function to send a array of markers, representing the curve, to rviz
    :param traj: trajectory list of points
    :param pub_rviz: ROS publisher object
    :return:
    """
    if not pub_marker_array:
        raise AssertionError("pub_marker_array is not valid:%s".format(pub_marker_array))

    #Create a MarkerArray message
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
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        # Color and transparency
        marker.color.a = 0.7
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        # Pose
        marker.pose.orientation.w = 1.0
        px = traj[0][i]
        py = traj[1][i]
        pz = traj[2][i]
        marker.pose.position.x = px
        marker.pose.position.y = py
        marker.pose.position.z = pz

        # Append marker to array
        points_marker.markers.append(marker)

    # Publish marker array
    pub_marker_array.publish(points_marker)#



#Callback to command the computation of a path
def callback_start(msg):
    global path, points, pub_traj, robot_position, pub_marker
    global N_points

    #Number of points to be interpolated
    N = N_points

    if(msg.data == True):
        #Call the planner
        path = planner(N, points, robot_position,pub_marker)

        #publish a polygon message with the path
        poly_msg = Polygon()
        for i in range(len(path[0])):
            point_msg = Point32()
            point_msg.x = path[0][i]
            point_msg.y = path[1][i]
            point_msg.z = path[2][i]

            poly_msg.points.append(point_msg)


        pub_traj.publish(poly_msg)

        print("Path published")




#Callback to listen to the robot's position
def callback_odom(msg):
    global robot_position
    robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z-0.4]



#Callback to listen clicked points
def callback_new_point(msg):
    global points
    points.append([msg.point.x, msg.point.y, msg.point.z])
    print("%d points" % len(points))



#Callback to listem for a command to clear the listenned points
def callback_clear(msg):
    global points
    if (msg.data == True):
        points = []
        print("Points cleaned")



# Main routine
def run():
    global freq
    global pub_rviz_ref, pub_rviz_pose
    global points, robot_position
    global pub_traj, pub_marker

    global clicked_point_topic_name, pose_topic_name, start_topic_name, clear_topic_name

    points = []

    rospy.init_node("trajectory_planner_example_traj")


    # pub_traj = rospy.Publisher("/ref_path", Polygon, queue_size=10)
    # pub_marker = rospy.Publisher("/vizual_path", MarkerArray, queue_size=10)
    pub_traj = rospy.Publisher(path_topic_name, Polygon, queue_size=10)
    pub_marker = rospy.Publisher(visualization_topic_name, MarkerArray, queue_size=10)


    # rospy.Subscriber("clicked_point", PointStamped, callback_new_point)
    # rospy.Subscriber("/ekf_odom_2", Odometry, callback_odom)
    # rospy.Subscriber("/start_planner", Bool, callback_start)
    # rospy.Subscriber("/clear_planner", Bool, callback_clear)
    rospy.Subscriber(clicked_point_topic_name, PointStamped, callback_new_point)
    rospy.Subscriber(pose_topic_name, Odometry, callback_odom)
    rospy.Subscriber(start_topic_name, Bool, callback_start)
    rospy.Subscriber(clear_topic_name, Bool, callback_clear)


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

    global N_interpolation, clicked_point_topic_name, pose_topic_name, start_topic_name, clear_topic_name
    #Default number of points to be sampled in between ech pair of points
    N_interpolation = 5

    #Name of the topic in which the path will be passed to the controller (type: Polygon)
    path_topic_name = "ref_path"
    #Name of the topic in which the path will be passed to Rviz (type: MarkerArray)
    visualization_topic_name = "vizual_path"
    #Default name of the topic in which the pose will be obtained (type: PointStamped)
    clicked_point_topic_name = "clicked_point"
    #Default name of the topic in which the pose will be obtained (type: Odometry)
    pose_topic_name = "ekf_odom_2"
    #Default name of the topic in which the pose will be obtained (type: Bool)
    start_topic_name = "start_planner"
    #Default name of the topic in which the pose will be obtained (type: Bool)
    clear_topic_name = "clear_planner"




    # Obtain the parameters
    try:
        N_points = int(rospy.get_param("/trajectory_planner/N_points"));
        path_topic_name = rospy.get_param("/trajectory_planner/path_topic_name");
        visualization_topic_name = rospy.get_param("/trajectory_planner/visualization_topic_name");
        clicked_point_topic_name = rospy.get_param("/trajectory_planner/clicked_point_topic_name");
        pose_topic_name = rospy.get_param("/trajectory_planner/pose_topic_name");
        start_topic_name = rospy.get_param("/trajectory_planner/start_topic_name");
        clear_topic_name = rospy.get_param("/trajectory_planner/clear_topic_name");

        print("\n\33[92mParameters loaded:\33[0m")
        print("\33[94mN_points: " +  str(N_points) +"\33[0m")
        print("\33[94mpath_topic_name: " +  str(path_topic_name) +"\33[0m")
        print("\33[94mvisualization_topic_name: " + str(visualization_topic_name) +"\33[0m")
        print("\33[94mclicked_point_topic_name: " +  str(clicked_point_topic_name) +"\33[0m")
        print("\33[94mpose_topic_name: " + str(pose_topic_name) +"\33[0m")
        print("\33[94mstart_topic_name: " + str(start_topic_name) +"\33[0m")
        print("\33[94mclear_topic_name: " + str(clear_topic_name) +"\33[0m")

    except:
        print "\33[41mProblem occurred when trying to read the parameters!: clicked_poly_planner.py\33[0m"


    try:
        run()
    except rospy.ROSInterruptException:
        pass

    rospy.sleep(.5)
