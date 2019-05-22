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
# from scipy.spatial.transform import Rotation
import numpy as np





# Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global x_n, y_n, theta_n


    #print data

    if (data.transforms[0].child_frame_id == "EspeleoRobo"):

        x_n = data.transforms[0].transform.translation.x  # posicao 'x' do robo no mundo
        y_n = data.transforms[0].transform.translation.y  # posicao 'y' do robo no mundo
        z_n = data.transforms[0].transform.translation.z  # posicao 'z' do robo no mundo
        x_q = data.transforms[0].transform.rotation.x
        y_q = data.transforms[0].transform.rotation.y
        z_q = data.transforms[0].transform.rotation.z
        w_q = data.transforms[0].transform.rotation.w
        #euler = euler_from_quaternion([x_q, y_q, z_q, w_q])


        R_vrep = Rotation.from_quat(np.array([x_q, y_q, z_q, w_q]))
        R_corr = Rotation.from_quat(np.array([0.0, 0.7071, 0.0, 0.7071]))
        #R_corr = Rotation.from_quat(np.array([-0.5000, 0.5000, -0.5000, 0.5000]))

        #R = R_vrep*R_corr

        #q = R.as_quat()
        #aaa = euler_from_quaternion(q)
        #print "[aaa] = [",aaa[0]*180/pi,", ",aaa[1]*180/pi,", ",aaa[2]*180/pi,"]"

        #print "R = \n", R.as_dcm()

        #theta_n = euler[2]  # orientaco 'theta' do robo no mundo ???????????????????????????


        br1 = tf.TransformBroadcaster()
        #     0.5000, 0.5000, -0.5000, 0.5000
        phi = 0
        #br1.sendTransform((3*0, 0, 0), (sin(phi/2), sin(phi/2), sin(phi/2), cos(phi/2)), rospy.Time.now(), "/EspeleoRobo_2", "EspeleoRobo")
        br1.sendTransform((3*0, 0, 0), (0.0, 0.7071, 0.0, 0.7071), rospy.Time.now(), "/EspeleoRobo_2", "EspeleoRobo")
        br2 = tf.TransformBroadcaster()
        br2.sendTransform((x_n, y_n, z_n), (x_q, y_q, z_q, w_q), rospy.Time.now(), "/EspeleoRobo", "world")

    return


# ----------  ----------  ----------  ----------  ----------





# Callback to get the cmv_vel for the espeleorobo
def callback_cmd_vel(data):

    global v, omega
    global last_cmd_vel_msg

    v = data.linear.x/1.0
    omega = data.angular.z/1.0

    print "[v, omega] = [", v,", ",omega,"]"

    last_cmd_vel_msg = rospy.get_rostime().to_sec()

    return
# ----------  ----------  ----------  ----------  ----------




# Callback to get the positions of the wheels
def callback_joints(data):

    global jointsPositions

    jointsPositions = data.jointsPositions

    return
# ----------  ----------  ----------  ----------  ----------





# Rotina p



# Rotina feedback linearization
def compute_velocities(v, omega, joints):

    #print v, " ", omega, "AAAA"
    global vel_vec

    if (rospy.get_rostime().to_sec() - last_cmd_vel_msg >= TIME_OUT):
        v = 0
        omega = 0
        #vel_vec = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        vel_vec = [0.85*vel_vec[k] for k in range(6)]

    else:
        VD = 1*v + (L/2.0)*omega
        VE = 1*v - (L/2.0)*omega
        VD = VD/R
        VE = VE/R
        vel_vec = [-VD, -VD, -VD, VE, VE, VE]
        rospy.get_rostime().to_sec() - last_cmd_vel_msg

    SMART_LEGS=  True #Control the middle legs independntly
    if SMART_LEGS:
        vel_vec[1] = sin(pi-joints[1])
        vel_vec[4] = sin(pi-joints[4])

        #print "joints = ", joints



    #print "vel_vec = ", vel_vec

    return vel_vec

# ----------  ----------  ----------  ----------  ----------




# Rotina primaria
def control_6_wheels():





    pub_velocity_cmds = rospy.Publisher("/ros_eposmcd/velocity_movement", MovementArray, queue_size=1) #declaracao do topico para comando de velocidade
    rospy.init_node("espeleo_control") #inicializa o no "este no"
    #rospy.Subscriber("/turtle1/cmd_vel", Twist, callback_cmd_vel) #declaracao do topico onde sera lido o estado do robo
    rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel) #declaracao do topico onde sera lido o estado do robo
    rospy.Subscriber("/ros_eposmcd/joints_positions", EspeleoJoints, callback_joints) #declaracao do topico onde sera lido o estado do robo
    # rospy.Subscriber("/tf", TFMessage, callback_pose)

    #Define uma variavel que controlar[a a frequencia de execucao deste no
    rate = rospy.Rate(freq)


    # O programa do no consiste no codigo dentro deste while
    while not rospy.is_shutdown(): #"Enquanto o programa nao ser assassinado"

        # Aplica o feedback linearization
        vel_vec = compute_velocities(v, omega, jointsPositions)

        #print "Hello"


        vel_6_cmd = MovementArray()
        for k in range(6):
            vel_cmd = Movement()
            vel_cmd.nodeID = k+1
            vel_cmd.velocity = vel_vec[k]
            #vel_cmd.gear_reduction = 1.0
            #print "\n\nvel_cmd = \n", vel_cmd
            vel_6_cmd.movement_command.append(vel_cmd)

        pub_velocity_cmds.publish(vel_6_cmd)



        #Espera por um tempo de forma a manter a frequencia desejada
        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------






# Funcao inicial
if __name__ == '__main__':



    # Frequencia de simulacao no stage
    global freq
    freq = 20.0  # Hz

    global L #Distance between the center wheels
    L = 0.4347
    global R #Radius of the wheel
    R = 0.1997

    global v, omega
    v = 0
    omega = 0
    global jointsPositions
    jointsPositions = [0, 0, 0, 0, 0, 0]

    global last_cmd_vel_msg
    last_cmd_vel_msg = 0
    global TIME_OUT
    TIME_OUT = 0.5

    global vel_vec
    vel_vec = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    try:
        control_6_wheels()
    except rospy.ROSInterruptException:
        pass
