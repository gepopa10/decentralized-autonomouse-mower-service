#!/usr/bin/env python3

import rospy
import numpy as np
import time
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from termcolor import colored
from quaternion import quaternion
from sliding_differentiator import slid_diff

import pathlib
folder = str(pathlib.Path(__file__).parent.absolute())

Results_File_Cont_Pos = open(folder +"/" + "/Results_File_Cont_Pos.txt","w+")
Results_File_Cont_Vel = open(folder +"/" + "/Results_File_Cont_Vel.txt","w+")
Results_File_NED_CON = open(folder +"/" + "/Results_File_NED_CON.txt","w+")

data_NED = {}
data_Con_StP = {}
x_gt, y_gt, z_gt, qx_gt, qz_gt, qw_gt, roll_gt, pitch_gt, yaw_gt, dx_gt, dy_gt, dz_gt = 0

def clean_shutdown():
	Results_File_Cont_Pos.close()
	Results_File_Cont_Vel.close()
	Results_File_NED_CON.close()
	print(colored("controller is stopped!","green"))


def callback_NED(data):
	global data_NED
	data_NED = json.loads(data.data)

def odo_cb(msg):
    odo = msg
    global x_gt = msg.pose.pose.position.x #ground truth
    global y_gt = msg.pose.pose.position.y
    global z_gt = msg.pose.pose.position.z
    global qx_gt = msg.pose.pose.orientation.x
    global qy_gt = msg.pose.pose.orientation.y
    global qz_gt = msg.pose.pose.orientation.z
    global qw_gt = msg.pose.pose.orientation.w
    global roll_gt, pitch_gt, yaw_gt = tf.transformations.euler_from_quaternion((qx_gt, qy_gt, qz_gt, qw_gt))

	global dx_gt = msg.twist.twist.linear.x #ground truth
	global dy_gt = msg.twist.twist.linear.y
	global dz_gt = msg.twist.twist.linear.z

def controller():

    kp_x = 0.1
    kpi_x = 0.0001
    kpd_x = 0.01
    kvp_x = 0.1
    kvpi_x = 0.0001
    kvpd_x = 0.01

    ei_pos = 0.0
    evi_pos = 0.0

    edx_var1 = 0.0
    edx_var2 = 0.0
    evdx_var1 = 0.0
    evdx_var2 = 0.0

    rot_max = np.pi/8.0
	drot_max = np.pi
	max_v = 1
    data_NED_received = False
    time_stmp = 0.0
    rospy.init_node("controller",anonymous=True)
    rate = rospy.Rate(30)
    rospy.on_shutdown(clean_shutdown)

    # pub_Control_SetPoints = rospy.Publisher("Con_StP", String, queue_size=1)
    # publish_data_Con_StP = String()

	pub_Control_SetPoints = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	publish_data_Con_StP = Twist()
    rospy.sleep(5.0)

    time_stmp_origin = float(rospy.get_time())
    while not rospy.is_shutdown():
	    rate.sleep()
	    #--------------------------------------------------------------------------------------------------------------------------
	    #(START) measuring the frequency of the controller
	    Period = float(rospy.get_time()) - time_stmp
	    Frequency = 1.0/Period
#	    print(colored(("Frequency of controller: %.2f Hz" %Frequency),"red"))
	    time_stmp = float(rospy.get_time())
	    #(END) measuring the frequency of the controller
	    #--------------------------------------------------------------------------------------------------------------------------
	    #(START) NED position set-points
	    time_traject = time_stmp - time_stmp_origin
		# goal
		x_StP = 1
		y_StP = 0
	    xd_StP = x_StP
	    yd_StP = y_StP
#		xd_StP = pos_NED_m[0]
#		yd_StP = pos_NED_m[1]
#		if ((time_traject > 30.0) and (time_traject < 40.0)):
#			xd_StP = pos_NED_m[0] + (0.1 * (time_traject - 30.0))
#			yd_StP = pos_NED_m[1]
#		if ((time_traject > 40.0) and (time_traject < 50.0)):
#			xd_StP = pos_NED_m[0]
#			yd_StP = pos_NED_m[1] + (0.1 * (time_traject - 40.0))
	    #(END) NED position set-points
	    #---------------------------------------------------------------------------------------------------------------------
	    #(START) subscribing to estimated position and velocity data
	    data_NED_received = False
	    # rospy.Subscriber("NED_POS_VEL", String, callback_NED, queue_size=1)
	    rospy.Subscriber("/odom", Odometry, odo_cb)
	    if not data_NED:
		    print(colored("Controller: No estimated position and velocity.","red"))
	    else:
		    data_NED_received = True
		    pos_NED_m[0] = data_NED.get("px")
		    pos_NED_m[1] = data_NED.get("py")
		    vel_NED_mps[0] = data_NED.get("vx")
		    vel_NED_mps[1] = data_NED.get("vy")
		    Results_File_NED_CON.write("%.6f %.3f %.3f %.3f %.3f \r\n" % (rospy.get_time(),pos_NED_m[0],pos_NED_m[1],vel_NED_mps[0],vel_NED_mps[1]))
	    #(END) subscribing to estimated position and velocity data
	    #--------------------------------------------------------------------------------------------------------------------------
		# if data_NED_received:
		if True:

		    dt = Period
		    # Des_yaw = atan2((yd_StP - pos_NED_m[1])/(xd_StP - pos_NED_m[0]))
		    # Des_pos = (xd_StP*cos(Des_psi)) + (yd_StP*sin(Des_psi))
		    # Act_pos = (vel_NED_mps[0]*cos(Des_psi)) + (vel_NED_mps[1]*sin(Des_psi))
		    # Act_vel = (vel_NED_mps[0]*cos(Des_psi)) + (vel_NED_mps[1]*sin(Des_psi))

		    Des_yaw = atan2((yd_StP - y_gt)/(xd_StP - x_gt))
		    Des_pos = (xd_StP*cos(Des_yaw)) + (yd_StP*sin(Des_yaw))
		    Act_pos = (x_gt*cos(yaw_gt)) + (y_gt*sin(yaw_gt))
		    Act_vel = (dx_gt*cos(yaw_gt)) + (dy_gt*sin(yaw_gt))

		    #(START) Position controller
		    ep_pos = Des_pos - Act_pos
		    ei_pos = ei_pos + ep_pos * dt
		    ed_pos,edx_var1,edx_var2 = slid_diff(ep_pos,edx_var1,edx_var2,Period)
		    vd_pos = kp_x * ep_pos + ki_x * ei_pos + kdx * ed_pos
		    v_pos_StP = np.clip(vd_pos,-max_v,max_v)
		    Results_File_Cont_Pos.write("%.6f %.3f %.3f %.3f\r\n" % (rospy.get_time(),ep_pos,vd_pos,v_pos_StP))
		    #(END) Position controller
		    #--------------------------------------------------------------------------------------------------------------------------
		    #(START) Velocity controller
		    # ev_pos = v_pos_StP - vel_NED_mps[0]
			ev_pos = v_pos_StP - vel_NED_mps[0]
		    evi_pos = evi_pos + ev_pos * dt
		    evd_pos,evdx_var1,evdx_var2 = slid_diff(ev_pos,evdx_var1,evdx_var2,Period)
		    thrust_cmd = - (kv_x * ev_pos + kvi_x * evi_pos + kvd_x * evd_pos)
		    thrust_StP = np.clip(thrust_cmd,-max_v,max_v)
		    yaw_StP = np.clip(Des_yaw,-rot_max,rot_max)
			dyaw_StP = np.clip(Des_dyaw,-drot_max,drot_max)
		    Results_File_Cont_Vel.write("%.6f %.3f %.3f %.3f\r\n" % (rospy.get_time(),ev_pos,thrust_cmd,thrust_StP))
		    quat_StP = quaternion(0.0,0.0,yaw_StP)
		    #(END) Velocity controller
		    #--------------------------------------------------------------------------------------------------------------------------
		    #(START) publishing the control setpoints
		    data_Con_StP = {'thrust_stp':thrust_StP,'quat_stp_w':quat_StP[0],'quat_stp_x':quat_StP[1],'quat_stp_y':quat_StP[2],'quat_stp_z':quat_StP[3]}
		    #publish_data_Con_StP.data = json.dumps(data_Con_StP)

			publish_data_Con_StP.vel_msg.linear.x = thrust_StP
			publish_data_Con_StP.vel_msg.linear.y = 0
			publish_data_Con_StP.vel_msg.linear.z = 0
			publish_data_Con_StP.vel_msg.angular.x = 0
			publish_data_Con_StP.vel_msg.angular.y = 0
			publish_data_Con_StP.vel_msg.angular.z = dyaw_StP
		    pub_Control_SetPoints.publish(publish_data_Con_StP)
		    #(END) publishing the control setpoints
		    #--------------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    try:
	    controller()
    except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
	    pass
