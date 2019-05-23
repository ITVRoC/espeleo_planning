#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
from tf2_msgs.msg import TFMessage
# from scipy.spatial.transform import Rotation
import numpy as np


# -----------------------------------
# ------ Ellipse parameters ------
# -----------------------------------
global a, b, cx, cy, phi
a = 3 # semiaxes x
b = 2 # semiaxes x
cx = 0 # center x
cy = 0 # center y
phi = pi / 4 # ellipse rotation angle
global cte_vel
cte_vel = 1 # cte_vel = 1 -> trajectory with constant velocity; cte_vel = 0 -> constant speed of the parameter
# Desired revolution time
global T
T = 40
# -----------------------------------
# -----------------------------------


# Obtainnment of the average speed along the ellipse
global Vd
Vd = 0
dp = 0.001
for plist in range(1, 6283, 1):
    p = (float(plist) - 1) / 1000
    Vd = Vd + sqrt((a * sin(p)) ** 2 + (b * cos(p)) ** 2) * dp
Vd = Vd / T
# Simulation frequency
global freq
freq = 10.0  # Hz

# Saturation speed
global Usat
Usat = 1.5

global x_n, y_n, theta_n
x_n = 0.1  # currrente x position of the robot
y_n = 0.2  # currrente y position of the robot
theta_n = 0.001  # currrente theta position of the robot

# Distance d for the feedback linearization controller
global d
d = 0.25
# Proportional gain for the controller
global Kp
Kp = 0.6

global p_last # additional variable to gnerate with a trajectory with constant velocity
p_last = 0;





# Callback to detect the robot pose
def callback_pose(data):
    global x_n, y_n, theta_n

    # For every transform in the message
    for T in data.transforms:

        # Check if the transform refers to the EspeleoRobo
        if (T.child_frame_id == "EspeleoRobo"):

            x_n = T.transform.translation.x # x position
            y_n = T.transform.translation.y # y position
            z_n = T.transform.translation.z # z position
            x_q = T.transform.rotation.x # quaternion
            y_q = T.transform.rotation.y # quaternion
            z_q = T.transform.rotation.z # quaternion
            w_q = T.transform.rotation.w # quaternion

            euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
            theta_n = euler[2]


    return
# ----------  ----------  ----------  ----------  ----------






# Routine to map the time t in the parameter p(t)
# This is used to obtain a trajectory with constant speed
def map_time_p(time):
    global freq
    global T, Vd
    global a, b
    global p_last

    # If a constant speed trajectory is wanted
    if(cte_vel == 1):
        dpdt = Vd/sqrt((a*sin(p_last))**2+(b*cos(p_last))**2)
        p = p_last + dpdt*(1/freq)
        p_last = p

    # If the original trajectory is wanted (the ispeed is coupled to the parametrization)
    else:
        T = 40 # cycle period
        p = 2*pi*time/T # parameter
        dpdt = 2*pi/T

    return p, dpdt
# ----------  ----------  ----------  ----------  ----------




# Function to generate a reference trajectory
def refference_trajectory(time):

    # Get the appropriate parameter'
    [p, dpdt] = map_time_p(time)

    # Reference for position without rotation and translation
    x_ref0 = a * cos(p)
    y_ref0 = b * sin(p)

    # Reference for position after rotation and translation
    x_ref = cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx * 1
    y_ref = sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy * 1

    # Derivative of the trajectory
    Vx_ref = cos(phi) * (-a * sin(p)) * dpdt - sin(phi) * (b * cos(p)) * dpdt
    Vy_ref = sin(phi) * (-a * sin(p)) * dpdt + cos(phi) * (b * cos(p)) * dpdt

    return (x_ref, y_ref, Vx_ref, Vy_ref)

# ----------  ----------  ----------  ----------  ----------




# Rotina para a geracao da entrada de controle
def trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref):
    global x_n, y_n, theta_n
    global Kp
    global Usat

    # Controller - proportional plus feedforward
    Ux = Vx_ref + Kp * (x_ref - x_n)
    Uy = Vy_ref + Kp * (y_ref - y_n)

    # Saturation of the control signal
    absU = sqrt(Ux ** 2 + Uy ** 2)
    if (absU > Usat):
        Ux = Usat * Ux / absU
        Uy = Usat * Uy / absU


    return (Ux, Uy)

# ----------  ----------  ----------  ----------  ----------




# Feedback linearization
def feedback_linearization(Ux, Uy):
    global x_n, y_n, theta_n
    global d

    # Transform (Vx, Vy) in (v, omega) using feedback linearization
    VX = cos(theta_n) * Ux + sin(theta_n) * Uy
    WZ = (-sin(theta_n) / d) * Ux + (cos(theta_n) / d) * Uy

    return (VX, WZ)

# ----------  ----------  ----------  ----------  ----------




# Routine to send the ellipse for the rviz
def send_ellipse_to_rviz():
    global a, b, cx, cy, phi

    points_marker = MarkerArray()
    marker = Marker()
    for p0 in range(1, 628, 1):
        #print "p0 = ", p0
        p = p0 / 100.0
        x = cos(phi) * (a * cos(p)) - sin(phi) * (b * sin(p)) + cx * 1
        y = sin(phi) * (a * cos(p)) + cos(phi) * (b * sin(p)) + cy * 1
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()
        marker.id = p0 - 1
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        #print "marker = ", marker
        points_marker.markers.append(marker)

    return (points_marker)
# ----------  ----------  ----------  ----------  ----------




# One more routine to bublish information to rviz
def send_marker_to_rviz(x_ref, y_ref, Vx_ref, Vy_ref, Ux, Uy):
    global x_n, y_n, theta_n
    global x_goal, y_goal
    global pub_rviz_ref, pub_rviz_pose

    mark_ref = Marker()
    mark_pose = Marker()

    mark_ref.header.frame_id = "/world"
    mark_ref.header.stamp = rospy.Time.now()
    mark_ref.id = 0
    mark_ref.type = mark_ref.ARROW
    mark_ref.action = mark_ref.ADD
    mark_ref.scale.x = 0.5 * (Vy_ref ** 2 + Vx_ref ** 2) ** (0.5)
    mark_ref.scale.y = 0.1
    mark_ref.scale.z = 0.1
    mark_ref.color.a = 1.0
    mark_ref.color.r = 1.0
    mark_ref.color.g = 1.0
    mark_ref.color.b = 1.0
    mark_ref.pose.position.x = x_ref
    mark_ref.pose.position.y = y_ref
    mark_ref.pose.position.z = 0.0
    quaternio = quaternion_from_euler(0, 0, atan2(Vy_ref, Vx_ref))
    mark_ref.pose.orientation.x = quaternio[0]
    mark_ref.pose.orientation.y = quaternio[1]
    mark_ref.pose.orientation.z = quaternio[2]
    mark_ref.pose.orientation.w = quaternio[3]

    mark_pose.header.frame_id = "/world"
    mark_pose.header.stamp = rospy.Time.now()
    mark_pose.id = 1
    mark_pose.type = mark_pose.ARROW
    mark_pose.action = mark_pose.ADD
    mark_pose.scale.x = 0.5 * sqrt(Ux ** 2 + Uy ** 2)
    mark_pose.scale.y = 0.1
    mark_pose.scale.z = 0.1
    mark_pose.color.a = 1.0
    mark_pose.color.r = 1.0
    mark_pose.color.g = 0.0
    mark_pose.color.b = 0.0
    mark_pose.pose.position.x = x_n
    mark_pose.pose.position.y = y_n
    mark_pose.pose.position.z = 0.1
    quaternio = quaternion_from_euler(0, 0, theta_n)
    mark_pose.pose.orientation.x = quaternio[0]
    mark_pose.pose.orientation.y = quaternio[1]
    mark_pose.pose.orientation.z = quaternio[2]
    mark_pose.pose.orientation.w = quaternio[3]

    pub_rviz_ref.publish(mark_ref)
    pub_rviz_pose.publish(mark_pose)

    return

# ----------  ----------  ----------  ----------  ----------




# Primary function
def ellipse():
    global freq
    global x_n, y_n, theta_n
    global pub_rviz_ref, pub_rviz_pose

    vel = Twist()

    i = 0

    # Publisher for a command velocity
    pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # Init node
    rospy.init_node("ellipse")
    # Subscribe to the position of the robot
    rospy.Subscriber("/tf", TFMessage, callback_pose)
    # Publisher to visualize the ellipse on rviz
    pub_rviz_ellipse = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)


    rate = rospy.Rate(freq)


    sleep(1)

    # Send ellipse to rviz
    pointsMarker = send_ellipse_to_rviz()
    pub_rviz_ellipse.publish(pointsMarker)

    while not rospy.is_shutdown():

        i = i + 1
        time = i / float(freq)

        # Get the trajectory
        [x_ref, y_ref, Vx_ref, Vy_ref] = refference_trajectory(time)

        # Compute control signals for x and y velocities
        [Ux, Uy] = trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref)

        # Compute control signals for v and omega
        [V_forward, w_z] = feedback_linearization(Ux, Uy)

        # Construct the message
        vel.linear.x = V_forward
        vel.angular.z = w_z

        # Publish message
        pub_stage.publish(vel)


        rate.sleep()
# ---------- !! ---------- !! ---------- !! ---------- !! ----------






# Funcao inicial
if __name__ == '__main__':

    try:
        ellipse()
    except rospy.ROSInterruptException:
        pass
