#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Polygon
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
from tf2_msgs.msg import TFMessage
# from scipy.spatial.transform import Rotation
import numpy as np
import copy


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



# Obtain the trajectory to be followed by the robot
def callback_trajectory(data):

    global traj
    global state_k
    global has_trajectory_flag
    global N

    # Define a list
    traj = [[],[]]

    # Iterate over the message to get all of the points of the curve
    for k in range(len(data.points)):
        p = data.points[k]
        traj[0].append(p.x)
        traj[1].append(p.y)
        #traj[[z],[k]] = p.z# z is not used

    # Update the closest index - index to the closest point in the curve
    N = len(traj[0])
    state_k = 0
    D = 100000
    for k in range(N):
        D_temp = sqrt((pos[0]-traj[0][k])**2 + (pos[1]-traj[1][k])**2)
        if (D_temp<D):
            state_k = k
            D = D_temp

    # Set flag indicating that the program already has a curve to be followed
    has_trajectory_flag = True


    return
# ----------  ----------  ----------  ----------  ----------




# Compute the vectro field that will guide the robot
def vec_field(pos):

    global state_k, state_k_delta
    global traj
    global N

    # Get x and y position
    x = pos[0]
    y = pos[1]

    # Compute the closest ponit on the curve
    # Consider only the points in the vicinity of the current closest point (robustness)
    k_vec = [state_k-state_k_delta+i for i in range(state_k_delta)]
    k_vec.append(state_k)
    k_vec = k_vec + [state_k+1+i for i in range(state_k_delta)]
    for k in range(len(k_vec)):
        if k_vec[k]<0:
            k_vec[k] = k_vec[k] + N
        if k_vec[k]>=N:
            k_vec[k] = k_vec[k] - N
    # Iterate over the indexes in the list k_vec to get the closest point
    D = 100000
    for k in k_vec:
        D_temp = sqrt((x-traj[0][k])**2 + (y-traj[1][k])**2)
        if (D_temp<D):
            k_min = k
            D = D_temp
    # Update state_k (index of the closest point)
    state_k = k_min

    # Compute the distance vector
    D_vec = [x-traj[0][k_min], y-traj[1][k_min]]
    # Compute the gradient of the distance Function
    grad_D = [D_vec[0]/(D+0.000001), D_vec[1]/(D+0.000001)]

    # Compute the tangent vector of the curve at k_min
    k1 = k_min - 1 # previous index
    if k1 == -1:
        k1 = N-1
    k2 = k_min + 1 # next index
    if k2 == N:
        k2 = 0
    # Numerically compute the tangent vector and normalize it
    T = [traj[0][k2]-traj[0][k1], traj[1][k2]-traj[1][k1]]
    norm_T = sqrt(T[0]**2 + T[1]**2)
    T = [T[0]/norm_T, T[1]/norm_T]

    # Lyapunov Function
    P = (0.5)*D**2
    # Gain functions
    G = -(2/pi)*atan(kf*sqrt(P)) # convergence
    H = sqrt(1-G**2) # circulation

    # Compute the field's componnents
    Vx = G*grad_D[0] + H*T[0]
    Vy = G*grad_D[1] + H*T[1]

    return (Vx, Vy)
# ----------  ----------  ----------  ----------  ----------



# Function feedback linearization
def feedback_linearization(Ux, Uy):
    global d

    # Get the yaw angle
    psi = rpy[2]

    # Compute foward velocity and angular velocity
    VX = cos(psi) * Ux + sin(psi) * Uy
    WZ = (-sin(psi) / d) * Ux + (cos(psi) / d) * Uy

    return (VX, WZ)
# ----------  ----------  ----------  ----------  ----------




# Function to send a array of markers, representing the curve, to rviz
def send_curve_to_rviz(pub_rviz):

    # Create messsage
    points_marker = MarkerArray()
    marker = Marker()
    # Iterate over the points
    for k in range(N):
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

    # Publish marker array
    pub_rviz.publish(points_marker)

    return (points_marker)
# ----------  ----------  ----------  ----------  ----------




# Function to send a markers, representing the value of the field
def send_marker_to_rviz(pub_rviz, Vx, Vy):

    mark_ref = Marker()

    mark_ref.header.frame_id = "/world"
    mark_ref.header.stamp = rospy.Time.now()
    mark_ref.id = 0
    mark_ref.type = mark_ref.ARROW
    mark_ref.action = mark_ref.ADD
    # Size of the marker
    mark_ref.scale.x = 1.5 * (Vy ** 2 + Vx ** 2) ** (0.5)
    mark_ref.scale.y = 0.08
    mark_ref.scale.z = 0.08
    # Collor and transparency
    mark_ref.color.a = 1.0
    mark_ref.color.r = 0.0
    mark_ref.color.g = 0.0
    mark_ref.color.b = 0.0
    # Position of the marker
    mark_ref.pose.position.x = pos[0]
    mark_ref.pose.position.y = pos[1]
    mark_ref.pose.position.z = pos[2]
    #Orientation of the marker
    quaternio = quaternion_from_euler(0, 0, atan2(Vy, Vx))
    mark_ref.pose.orientation.x = quaternio[0]
    mark_ref.pose.orientation.y = quaternio[1]
    mark_ref.pose.orientation.z = quaternio[2]
    mark_ref.pose.orientation.w = quaternio[3]

    # Publish marker
    pub_rviz_ref.publish(mark_ref)

    return

# ----------  ----------  ----------  ----------  ----------




# Rotina primaria
def vector_field():
    global freq
    global x_n, y_n, theta_n
    global pub_rviz_ref, pub_rviz_pose
    global traj
    global N

    vel = Twist()

    i = 0

    # Publisher for Twist message
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # Init node
    rospy.init_node("vector_field")
    # Subscribers
    rospy.Subscriber("/tf", TFMessage, callback_pose) # ground thruth
    rospy.Subscriber("/espeleo/traj_points", Polygon, callback_trajectory) # points of the curve to be followed

    # Publishers for rviz
    pub_rviz_ref = rospy.Publisher("/visualization_marker_ref", Marker, queue_size=1) #rviz marcador de velocidade de referencia
    pub_rviz_curve = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1) #rviz array de marcadores no espaco da elipse

    rate = rospy.Rate(freq)

    # Wait until a curve is received
    while not has_trajectory_flag and not rospy.is_shutdown():
        #print "Waitting for trajectory ..."
        rate.sleep()


    #Loop
    while not rospy.is_shutdown():

        # Count time
        i = i + 1
        time = i / float(freq)

        # Try to compute the vector field
        # It may fail when the program is receiving a new sequence of points
        try:

            # Compute field
            [Vx_ref, Vy_ref] = vec_field(pos)

            # Compute a command of velocity
            [V_forward, w_z] = feedback_linearization(Vx_ref, Vy_ref)

            # Atribute values to the Twist message
            vel.linear.x = V_forward
            vel.angular.z = w_z

            # Publish velocity
            pub_cmd_vel.publish(vel)

            # Send markers to rviz
            # send_curve_to_rviz(pub_rviz_curve) # Not necessary
            send_marker_to_rviz(pub_rviz_ref, Vx_ref, Vy_ref)

        except:
            # This is due to the changes in the curve's change
            print "Temporary problem in the computation of the field !"



        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------






# Funcao inicial
if __name__ == '__main__':

    # Frequency of field computation
    global freq
    freq = 100.0  # Hz

    # Robot position and orientation
    global pos, rpy
    pos = [0, 0, 0]
    rpy = [0, 0, 0]

    # Convergence intensity of the vector field
    global kf
    kf = 5.0

    # Constant relative to the feedback linearization controller
    global d
    d = 0.2

    # Flag to initialize the vector field after a curve is received
    global has_trajectory_flag
    has_trajectory_flag = False

    # Variable
    global state_k, state_k_delta
    state_k = 0
    state_k_delta = 10


    # List of points of the curve
    global traj
    # Number of points in the curve
    global N


    try:
        vector_field()
    except rospy.ROSInterruptException:
        pass
