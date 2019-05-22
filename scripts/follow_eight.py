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
# ------ Parametros da ellipse ------
# -----------------------------------
global a, b, cx, cy, phi
a = 2 #semieixo em x
b = 1 #semieixo em y
cx = 0 #x do centro
cy = 0 #y do centro
phi = 0*pi / 4 #angulo de rotacao (rad)
global cte_vel
cte_vel = 1 # cte_vel = 1 -> trajetoria com velocidade constante; cte_vel = 0 -> parametro com velocidade constante
# Tempo de revolucao desejado
global T
T = 60
# -----------------------------------
# -----------------------------------


# Obtencao da velocidade media concordante com a, b e T
global Vd
Vd = 0
dp = 0.001
for plist in range(1, 6283, 1):
    p = (float(plist) - 1) / 1000
    Vd = Vd + sqrt((a * sin(p)) ** 2 + (b * sin(2.0*p)) ** 2) * dp
Vd = Vd / T
# Frequencia de simulacao no stage
global freq
freq = 10.0  # Hz


# Velocidade de saturacao
global Usat
Usat = 4

global x_n, y_n, theta_n
x_n = 0.1  # posicao x atual do robo
y_n = 0.2  # posicao y atual do robo
theta_n = 0.001  # orientacao atual do robo

# Relativo ao feedback linearization
global d
d = 0.2
# Relativo ao controlador (feedforward + ganho proporcional)
global Kp
Kp = 0.5

global p_last #variavel adicional para gerar trajetoria com velocidade constante
p_last = 0;




# Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global x_n, y_n, theta_n


    #print data
    if (data.transforms[0].child_frame_id == "EspeleoRobo"):

        x_n = data.transforms[0].transform.translation.x  # posicao 'x' do robo no mundo
        y_n = data.transforms[0].transform.translation.y  # posicao 'y' do robo no mundo
        x_q = data.transforms[0].transform.rotation.x
        y_q = data.transforms[0].transform.rotation.y
        z_q = data.transforms[0].transform.rotation.z
        w_q = data.transforms[0].transform.rotation.w
        euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
        theta_n = euler[2]

        #
        # R_vrep = Rotation.from_quat(np.array([x_q, y_q, z_q, w_q]))
        # R_corr = Rotation.from_quat(np.array([0.0, 0.7071, 0.0, 0.7071]))
        #
        # R = R_vrep*R_corr
        # # R = R_vrep*R_corr_2*R_corr
        # # R = R_corr*R_vrep
        #
        # q = R.as_quat()
        # #print euler_from_quaternion(q)
        # euler = euler_from_quaternion(q)
        # theta_n = euler[2]
        # #print "theta_n = ", theta_n*180/pi
        #
        # #print "R_corr = \n", R_corr.as_dcm()
        #
        # #theta_n = euler[1]  # orientaco 'theta' do robo no mundo ???????????????????????????
        #
        # print "[x_n, y_n, theta_n] = ", x_n, " ", y_n, " ",theta_n*180/pi
        #
        # print "R = \n", R.as_dcm().tolist()[0]
        # print "", R.as_dcm().tolist()[1]
        # print "", R.as_dcm().tolist()[2]



    return


# ----------  ----------  ----------  ----------  ----------






# Rotina para mapear o tempo no parametro p
def map_time_p(time):
    global freq
    global T, Vd
    global a, b
    global p_last

    if(cte_vel == 1):
        #dpdt = Vd/sqrt((a*sin(p_last))**2+(b*cos(p_last))**2)
        dpdt = Vd/sqrt((a*cos(p_last))**2+(b*2.0*cos(2.0*p_last))**2)
        p = p_last + dpdt*(1/freq)
        p_last = p
    else:
        T = 40 #periodo de ciclo
        p = 2*pi*time/T #parametro
        dpdt = 2*pi/T

    return p, dpdt
# ----------  ----------  ----------  ----------  ----------




# Rotina para a geracao da trajetoria de referencia
def refference_trajectory(time):

    [p, dpdt] = map_time_p(time)

    x_ref0 = a * sin(p)
    y_ref0 = b * sin(2.0*p)

    vx_ref0 = a * cos(p)*dpdt
    vy_ref0 = b * 2.0 * cos(2.0*p)*dpdt

    # H = [cos(phi) -sin(phi) cx
    #     sin(phi)  cos(phi) cy
    #     0         0        1]
    x_ref = cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx * 1
    y_ref = sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy * 1

    Vx_ref = cos(phi) * vx_ref0 - sin(phi) * vy_ref0
    Vy_ref = sin(phi) * vx_ref0 + cos(phi) * vy_ref0

    #x_ref = 2.0
    #y_ref = 2.0
    #Vx_ref = 0.0
    #Vy_ref = 0.0

    #global Vd
    #print "Modulo de V_d   -> ", Vd
    #print "Modulo de V_ref -> ", sqrt(Vx_ref ** 2 + Vy_ref ** 2)
    #print "Conta de V_ref  -> ", sqrt((a*sin(p))**2 + (b*cos(p))**2)*dpdt

    return (x_ref, y_ref, Vx_ref, Vy_ref)

# ----------  ----------  ----------  ----------  ----------




# Rotina para a geracao da entrada de controle
def trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref):
    global x_n, y_n, theta_n
    global Kp
    global Usat

    Ux = Vx_ref + Kp * (x_ref - x_n)
    Uy = Vy_ref + Kp * (y_ref - y_n)

    # print "erro_pos = ", (x_ref - x_n), " ", (y_ref - y_n)

    absU = sqrt(Ux ** 2 + Uy ** 2)

    if (absU > Usat):
        Ux = Usat * Ux / absU
        Uy = Usat * Uy / absU

    # print "Ux, Uy = ", Ux, Uy

    return (Ux, Uy)

# ----------  ----------  ----------  ----------  ----------




# Rotina feedback linearization
def feedback_linearization(Ux, Uy):
    global x_n, y_n, theta_n
    global d

    VX = cos(theta_n) * Ux + sin(theta_n) * Uy
    WZ = (-sin(theta_n) / d) * Ux + (cos(theta_n) / d) * Uy

    # print "Ux, Uy = ", Ux, Uy

    return (VX, WZ)

# ----------  ----------  ----------  ----------  ----------




# Rotina executada apenas uma vez para mostrar a ellipse no rviz
def send_eight_to_rviz():
    global a, b, cx, cy, phi

    points_marker = MarkerArray()
    marker = Marker()
    for p0 in range(1, 628, 1):
        #print "p0 = ", p0
        p = p0 / 100.0
        #x = cos(phi) * (a * cos(p)) - sin(phi) * (b * sin(p)) + cx * 1
        #y = sin(phi) * (a * cos(p)) + cos(phi) * (b * sin(p)) + cy * 1
        x = cos(phi) * (a * sin(p)) - sin(phi) * (b * sin(2.0*p)) + cx * 1
        y = sin(phi) * (a * sin(p)) + cos(phi) * (b * sin(2.0*p)) + cy * 1
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()
        marker.id = p0 - 1
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        #print "marker = ", marker
        points_marker.markers.append(marker)

    return (points_marker)
# ----------  ----------  ----------  ----------  ----------




# Rotina para piblicar informacoes no rviz
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
    mark_ref.scale.x = 1.5 * (Vy_ref ** 2 + Vx_ref ** 2) ** (0.5)
    mark_ref.scale.y = 0.08
    mark_ref.scale.z = 0.08
    mark_ref.color.a = 1.0
    mark_ref.color.r = 0.0
    mark_ref.color.g = 0.0
    mark_ref.color.b = 0.0
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
    #pub_rviz_pose.publish(mark_pose)

    return

# ----------  ----------  ----------  ----------  ----------




# Rotina primaria
def ellipse():
    global freq
    global x_n, y_n, theta_n
    global pub_rviz_ref, pub_rviz_pose

    vel = Twist()

    i = 0

    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0

    pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.init_node("ellipse")
    rospy.Subscriber("/tf", TFMessage, callback_pose)

    pub_rviz_ref = rospy.Publisher("/visualization_marker_ref", Marker, queue_size=1) #rviz marcador de velocidade de referencia
    # pub_rviz_pose = rospy.Publisher("/visualization_marker_pose", Marker, queue_size=1) #rviz marcador de velocidade do robo
    pub_rviz_eight = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1) #rviz array de marcadores no espaco da elipse

    rate = rospy.Rate(freq)



    sleep(1)

    while not rospy.is_shutdown():

        i = i + 1
        time = i / float(freq)

        [x_ref, y_ref, Vx_ref, Vy_ref] = refference_trajectory(time)
        #print "[x_ref, y_ref] = ", x_ref, y_ref
        #print "[Vx_ref, Vy_ref] = ", Vx_ref, Vy_ref

        [Ux, Uy] = trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref)
        #print "[Ux, Uy] = ", Ux, Uy

        [V_forward, w_z] = feedback_linearization(Ux, Uy)
        #print "[V_forward, w_z] = ", V_forward, w_z

        #print "time", time
        #V_forward = 0.2
        #w_z = 0.0

        #print "[V_forward, w_z] = [", V_forward, ", ", w_z,"]"

        #print "---------- ---------- ----------"

        send_marker_to_rviz(x_ref, y_ref, Vx_ref, Vy_ref, Ux, Uy)

        vel.linear.x = V_forward
        vel.angular.z = w_z

        pub_stage.publish(vel)


        pointsMarker = send_eight_to_rviz()
        pub_rviz_eight.publish(pointsMarker)



        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------






# Funcao inicial
if __name__ == '__main__':

    try:
        ellipse()
    except rospy.ROSInterruptException:
        pass
