#!/usr/bin/env python3

'''
Universidade Federal de Minas Gerais (UFMG) - 2020
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
Victor R. F. Miranda, <victormrfm@gmail.com>
João F. R. Baiao, <baiaojfr.eng@gmail.com>
Gilmar P. C. Junior, <gilmarpcruzjunior@gmail.com>

Last update: Nov, 01 2023
'''

import rospy

import numpy as np

# from scipy import linalg as LA

from geometry_msgs.msg import PointStamped, Polygon, Point32
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

from espeleo_control.msg import Path


class Planner:
    def __init__(self):
        # Frequency of the loop
        self.freq = 10.0  # Hz

        self.robot_position = [0, 0, 0]
        self.points = []

        rospy.init_node("trajectory_planner_example_traj")

        self.read_params()

    def read_params(self):

        # Obtain the parameters
        try:
            self.N_points = int(rospy.get_param("~N_points", 8))
            self.path_topic_name = rospy.get_param("~path_topic_name", "ref_path")
            self.visualization_topic_name = rospy.get_param("~visualization_topic_name", "visual_path")
            self.clicked_point_topic_name = rospy.get_param("~clicked_point_topic_name", "clicked_point")
            self.visualization_points_topic_name = rospy.get_param("~visualization_points_topic_name", "visual_points")
            self.pose_topic_name = rospy.get_param("~pose_topic_name", "ekf_odom_2")
            self.path_frame_id = rospy.get_param("~path_frame_id", "os1_init")
            # self.node_name = rospy.get_param("/node_name", "trajectory_planner_example_traj")

            self.start_topic_name = rospy.get_param("~start_topic_name", "/start_planner")
            self.clear_topic_name = rospy.get_param("~clear_topic_name", "/clear_planner")

            print("\n\33[92mParameters loaded:\33[0m")
            print("\33[94mN_points: " +  str(self.N_points) +"\33[0m")
            print("\33[94mpath_topic_name: " +  str(self.path_topic_name) +"\33[0m")
            print("\33[94mvisualization_topic_name: " + str(self.visualization_topic_name) +"\33[0m")
            print("\33[94mclicked_point_topic_name: " +  str(self.clicked_point_topic_name) +"\33[0m")
            print("\33[94mpose_topic_name: " + str(self.pose_topic_name) +"\33[0m")        
            print("\33[94mpath_frame_id: " + str(self.path_frame_id) +"\33[0m")            
            # print("\33[94mnode_name: " + str(self.node_name) +"\33[0m")            
            print("\33[94mstart_topic_name: " + str(self.start_topic_name) +"\33[0m")
            print("\33[94mclear_topic_name: " + str(self.clear_topic_name) +"\33[0m")    


        except:
            print ("\33[41mProblem occurred when trying to read the parameters!: clicked_poly_planner.py\33[0m")

    # Main routine
    def run(self):

        # Publishers and Subscribers
        self.pub_traj = rospy.Publisher(self.path_topic_name, Path, queue_size=10)
        self.pub_marker_array = rospy.Publisher(self.visualization_topic_name, MarkerArray, queue_size=10)
        self.pub_points = rospy.Publisher(self.visualization_points_topic_name, MarkerArray, queue_size=10)

        rospy.Subscriber(self.clicked_point_topic_name, PointStamped, self.callback_new_point)
        rospy.Subscriber(self.pose_topic_name, Odometry, self.callback_odom)
        rospy.Service(self.start_topic_name, SetBool, self.handle_start)
        rospy.Service(self.clear_topic_name, SetBool, self.handle_clear)

        # Wait a bit
        rate = rospy.Rate(self.freq)

        while not rospy.is_shutdown():
        # Wait a bit
            rate.sleep()

    # ---------- !! ---------- !! ---------- !! ---------- !! ----------

    # Callback to robot's position
    def callback_odom(self, msg):
        self.robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

    # Callback to listen clicked points
    def callback_new_point(self, msg):
        self.points.append([msg.point.x, msg.point.y, msg.point.z])
        self.send_clicked_points_array_to_rviz()
        print("%d points" % len(self.points))

    # Callback to listen for the publish points command
    def handle_start(self, req):
        if(req.data == True):
            if len(self.points) != 0:
                # Call the planner
                self.planner()

                # Publish a polygon message with the path
                poly_msg = Path()
                for i in range(len(self.path[0])):
                    point_msg = Point32()
                    point_msg.x = self.path[0][i]
                    point_msg.y = self.path[1][i]
                    point_msg.z = self.path[2][i]

                    poly_msg.path.points.append(point_msg)

                poly_msg.header.stamp = rospy.Time.now()
                poly_msg.closed_path_flag = False
                poly_msg.insert_n_points = 8
                poly_msg.filter_path_n_average = 4
                
                self.pub_traj.publish(poly_msg)

                print("Path published")
                return SetBoolResponse(True, "Path published.")
            else:
                return SetBoolResponse(False, "No points to interpolate.")


    # Callback to listen for the clear points command
    def handle_clear(self, req):
        if (req.data == True):
            self.points = []
            print("Points cleaned")
            return SetBoolResponse(True, "Points cleaned.")

    # ---------- !! ---------- !! ---------- !! ---------- !! ----------

    # Function to obtain a polynomial interpolation
    def planner(self):

        # Include the current robot's position in the planning
        self.points = [self.robot_position] + self.points

        n = len(self.points)

        # Matrix associated to constrainned 3rd order polynomials
        M = [[0, 0, 0, 1],[1, 1, 1, 1],[0, 0, 1, 0],[3, 2, 1, 0]]
        Minv = [[2, -2, 1, 1],[-3, 3, -2, -1],[0, 0, 1, 0],[1, 0, 0, 0]]

        # Heurietics to define the 
        T = []
        for k in range(n):
            if k == 0:
                v = [self.points[k+1][0]-self.points[k][0], self.points[k+1][1]-self.points[k][1], self.points[k+1][2]-self.points[k][2]]
            elif k==(n-1):
                v = [self.points[k][0]-self.points[k-1][0], self.points[k][1]-self.points[k-1][1], self.points[k][1]-self.points[k-1][2]]
            else:
                v = [(self.points[k+1][0]-self.points[k-1][0])/2.0, (self.points[k+1][1]-self.points[k-1][1])/2.0, (self.points[k+1][1]-self.points[k-1][2])/2.0]
            T.append(v)

        # Definition of a vector to sample some points of the computed polynomials
        s_vec = [i/float(self.N_points) for i in range(self.N_points)]
        self.path = [[],[],[]]

        # Iterate over each pair of points
        for k in range(n-1):

            # Write the matrix that contains the boundary conditions for the polynomials
            A = [[self.points[k][0],self.points[k][1]], [self.points[k+1][0],self.points[k+1][1]], [T[k][0],T[k][1]], [T[k+1][0],T[k+1][1]]]
            #A = [[points[k][0],points[k][1],points[k][2]], [points[k+1][0],points[k+1][1],points[k+1][2]], [T[k][0],T[k][1],T[k][2]], [T[k+1][0],T[k+1][1],T[k+1][2]]]

            # Compute the coefficients
            ck = np.matrix(Minv)*np.matrix(A)
            
            # Sample the computed polynomials
            for s in s_vec:
                self.path[0].append(ck[0,0]*s**3+ck[1,0]*s**2+ck[2,0]*s**1+ck[3,0]*s**0)
                self.path[1].append(ck[0,1]*s**3+ck[1,1]*s**2+ck[2,1]*s**1+ck[3,1]*s**0)
                self.path[2].append(self.points[k][2]*(1.0-s)+self.points[k+1][2]*(s))
                #path[2].append(ck[0,2]*s**3+ck[1,2]*s**2+ck[2,2]*s**1+ck[3,2]*s**0)

        # Include the last point on the path
        self.path[0].append(ck[0,0]+ck[1,0]+ck[2,0]+ck[3,0])
        self.path[1].append(ck[0,1]+ck[1,1]+ck[2,1]+ck[3,1])
        self.path[2].append(self.points[k+1][2])
        #path[2].append(ck[0,2]+ck[1,2]+ck[2,2]+ck[3,2])

        # Publish the computed path to be visualized on rviz
        self.send_marker_array_to_rviz()


    # Function to send a marker array to rviz
    def send_marker_array_to_rviz(self):
        """Function to send a array of markers, representing the curve, to rviz
        :param traj: trajectory list of points
        :param pub_rviz: ROS publisher object
        :return:
        """
        if not self.pub_marker_array:
            raise AssertionError("pub_marker_array is not valid:%s".format(self.pub_marker_array))

        # Create a MarkerArray message
        points_marker = MarkerArray()
        for i in range(len(self.path[0])):
            marker = Marker()
            marker.header.frame_id = self.path_frame_id
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
            px = self.path[0][i]
            py = self.path[1][i]
            pz = self.path[2][i]
            marker.pose.position.x = px
            marker.pose.position.y = py
            marker.pose.position.z = pz

            # Append marker to array
            points_marker.markers.append(marker)

        # Publish marker array
        self.pub_marker_array.publish(points_marker)


    def send_clicked_points_array_to_rviz(self):
        """Function to send a array of markers, representing the clicked points, to rviz
        :param traj: trajectory list of points
        :param pub_rviz: ROS publisher object
        :return:
        """
        if not self.pub_points:
            raise AssertionError("pub_marker_array is not valid:%s".format(self.pub_points))

        #Create a MarkerArray message
        points_marker = MarkerArray()
        for i in range(len(self.points)):
            marker = Marker()
            marker.header.frame_id = self.path_frame_id
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            # Duration
            # marker.lifetime = rospy.Duration.from_sec(1.0)
            # Size of sphere
            marker.scale.x = 0.30 #0.15 
            marker.scale.y = 0.30 #0.15
            marker.scale.z = 0.30 #0.15
            # Color and transparency
            marker.color.a = 0.7
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            # Pose
            marker.pose.orientation.w = 1.0
            px = self.points[i][0]
            py = self.points[i][1]
            pz = self.points[i][2]
            marker.pose.position.x = px
            marker.pose.position.y = py
            marker.pose.position.z = pz

            # Append marker to array
            points_marker.markers.append(marker)

        # Publish marker array
        self.pub_points.publish(points_marker)

    # ---------- !! ---------- !! ---------- !! ---------- !! ----------



# Main function
if __name__ == '__main__':

    try:
        object = Planner()
        object.run()
    except rospy.ROSInterruptException:
        pass

    rospy.sleep(.5)
