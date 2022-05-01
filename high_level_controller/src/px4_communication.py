#!/usr/bin/env python3
import rospy
import serial
import json
import numpy as np
from pymavlink import mavutil
from std_msgs.msg import String
from termcolor import colored

PX4_connection = mavutil.mavlink_connection("/dev/ttyACM0",115200)
PX4_connection.wait_heartbeat()
print(colored(("PX4 Heartbeat(%u,%u,%u,%s,%u)" % (PX4_connection.mav_type, PX4_connection.target_system, PX4_connection.target_component, PX4_connection.flightmode, PX4_connection.base_mode)),"yellow"))

Results_File_EKF = open(r"/root/coop_con_loc_nano/src/coop_con_loc/src/Results_File_EKF.txt","w+")
Results_File_ACC = open(r"/root/coop_con_loc_nano/src/coop_con_loc/src/Results_File_ACC.txt","w+")
Results_File_NED = open(r"/root/coop_con_loc_nano/src/coop_con_loc/src/Results_File_NED.txt","w+")

data_NED = {}
data_ACC = {}
data_Att_StP = {}

def clean_shutdown():
	Results_File_EKF.close()
	Results_File_ACC.close()
	Results_File_NED.close()
	current_state_msg = PX4_connection.recv_match(type='HEARTBEAT',blocking=True)
	while(current_state_msg.system_status is not 3):
		PX4_connection.mav.command_long_send(PX4_connection.target_system,PX4_connection.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
		current_state_msg = PX4_connection.recv_match(type='HEARTBEAT',blocking=True)
	print(colored("Disarmed.","yellow"))
	print(colored("The connection to the PX4 is terminated!","yellow"))

def callback_NED(data):
	global data_NED
	data_NED = json.loads(data.data)

def callback_Att_StP(data):
	global data_Att_StP
	data_Att_StP = json.loads(data.data)

def px4_communication():

	pos_NED_m = [0.0,0.0,0.0]
	vel_NED_mps = [0.0,0.0,0.0]
	Acc_m = [0.0,0.0,0.0]
	Acc_bias_x_m = 0.0
	Acc_bias_y_m = 0.0
	Acc_bias_z_m = 9.8
	time_stmp = 0.0
	data_NED_received = False
	data_Att_StP_received = False
	rospy.init_node("px4_communication", anonymous=True)
	rate = rospy.Rate(120)
	rospy.on_shutdown(clean_shutdown)
	while not rospy.is_shutdown():		
		rate.sleep()
		Period = float(rospy.get_time()) - time_stmp
		Frequency = 1.0/Period
		print(colored(("PX4 frequency : %0.2f" % Frequency),"yellow"))
		time_stmp = float(rospy.get_time())
		#--------------------------------------------------------------------------------------------------------------------------
		#(START) publishing PX4 measured acceleration data
#		PX4_ACC_msg = PX4_connection.recv_match(type='HIGHRES_IMU',blocking=True)
#		if not PX4_ACC_msg:
#			print(colored("PX4 IMU data: no message is received!","red"))
#		else:
#			PX4_BIAS_msg = PX4_connection.recv_match(type='SENSOR_BIAS',blocking=False)
#			if PX4_BIAS_msg:
#				Acc_bias_x_m = PX4_BIAS_msg.axBias
#				Acc_bias_y_m = PX4_BIAS_msg.ayBias
#				Acc_bias_z_m = PX4_BIAS_msg.azBias
#			Acc_m[0] = PX4_ACC_msg.xacc + Acc_bias_x_m
#			Acc_m[1] = PX4_ACC_msg.yacc + Acc_bias_y_m
#			Acc_m[2] = PX4_ACC_msg.zacc + Acc_bias_z_m
#			#print("PX4 IMU data: %s" % PX4_ACC_msg)
#			Results_File_ACC.write("%d %.6f %.6f %.6f\r\n" % (rospy.get_time(),Acc_m[0],Acc_m[1],Acc_m[2]))
#			data_ACC = {'ax':Acc_m[0] , 'ay':Acc_m[1] , 'az':Acc_m[2]}
#			publish_acc_data.data = json.dumps(data_ACC)
#			pub_acc.publish(publish_acc_data)
		#(END) publishing PX4 measured acceleration data
		#--------------------------------------------------------------------------------------------------------------------------
		#(START) subscribing to estimated local position and velocity data
		rospy.Subscriber("NED_POS_VEL", String, callback_NED, queue_size=1)
		if not data_NED:
			print(colored("No estimated position and velocity.","red"))
		else:
			data_NED_received = True
			pos_NED_m[0] = data_NED.get("px")
			pos_NED_m[1] = data_NED.get("py")
			pos_NED_m[2] = data_NED.get("pz")
			vel_NED_mps[0] = data_NED.get("vx")
			vel_NED_mps[1] = data_NED.get("vy")
			vel_NED_mps[2] = data_NED.get("vz")
			Results_File_NED.write("%.6f %.3f %.3f %.3f %.3f %.3f %.3f\r\n" % (rospy.get_time(),pos_NED_m[0],pos_NED_m[1],pos_NED_m[2],vel_NED_mps[0],vel_NED_mps[1],vel_NED_mps[2]))
			print(colored(("estimated NED position (m): px=%.3f , py=%.3f , pz=%.3f" % (pos_NED_m[0],pos_NED_m[1],pos_NED_m[2])),"green"))
			print(colored(("estimated NED velocity (m/sec): vx=%.3f , vy=%.3f , vz=%.3f" % (vel_NED_mps[0],vel_NED_mps[1],vel_NED_mps[2])),"blue"))
		#(END) subscribing to estimated local position and velocity data
		#--------------------------------------------------------------------------------------------------------------------------
		#(START) subscribing to attitude set-points		
		thrust_stp = 0.0
		quat_stp = [0.0]*4
		rospy.Subscriber("Att_StP", String, callback_Att_StP, queue_size=1)
		if not data_Att_StP:
			print(colored("PX4: No attitude set-point is received.","red"))
		else:
			data_Att_StP_received = True
			thrust_stp = data_Att_StP.get("thrust_stp")
			quat_stp[0] = data_Att_StP.get("quat_stp_w")
			quat_stp[1] = data_Att_StP.get("quat_stp_x")
			quat_stp[2] = data_Att_StP.get("quat_stp_y")
			quat_stp[3] = data_Att_StP.get("quat_stp_z")
			print(colored(("attitude set-points: thrust:%.3f and quat:" % thrust_stp),"magenta"))
			print(colored(quat_stp,"magenta"))
		#(END) subscribing to attitude set-points
		#--------------------------------------------------------------------------------------------------------------------------
		#(START) transmitting the estimated local position and velocity to PX4
#		Pos_Covariance = [0]*21
#		Pos_Covariance[0] = 0.01
#		Pos_Covariance[6] = 0.01
#		Pos_Covariance[12] = 0.01
#		PX4_connection.mav.vision_position_estimate_send(PX4_ACC_msg.time_usec,pos_NED_m[0],pos_NED_m[1],pos_NED_m[2],0.0,0.0,0.0,Pos_Covariance)
#		Vel_Covariance = [0]*9
#		Vel_Covariance[0] = 0.01
#		Vel_Covariance[4] = 0.01
#		Vel_Covariance[8] = 0.01
#		PX4_connection.mav.vision_speed_estimate_send(PX4_ACC_msg.time_usec,vel_NED_mps[0],vel_NED_mps[1],vel_NED_mps[2],Vel_Covariance)
		#(END) transmitting the estimated local position and velocity to PX4
		#-------------------------------------------------------------------------------------------------------------------------
		#(START) logging the local NED position and velocity from PX4
#		PX4_msg = PX4_connection.recv_match(type='LOCAL_POSITION_NED',blocking=True)
#		#PX4_Con_msg = PX4_connection.recv_match(type='CONTROL_SYSTEM_STATE',blocking=False)
#		if not PX4_msg:
#			print(colored("PX4 local_NED data: no message is received!","red"))
#		else:
#			print(colored(("PX4 local_NED data: %s" % PX4_msg),"yellow"))
#			Results_File_EKF.write("%d %.6f %.6f %.6f %.6f %.6f %.6f\r\n" % (rospy.get_time(),PX4_msg.x,PX4_msg.y,PX4_msg.z,PX4_msg.vx,PX4_msg.vy,PX4_msg.vz))
		#(END) logging the local NED position and velocity from PX4
		#------------------------------------------------------------------------------------------------------------------------	
		if data_Att_StP_received:
			#(START) transmitting the target command to PX4
			Result_Set = PX4_connection.mav.set_attitude_target_send(int(rospy.get_time()),PX4_connection.target_system,PX4_connection.target_component,0b00000111,quat_stp,0.0,0.0,0.0,thrust_stp) #PX4_msg.time_boot_ms
			#Result_Set = PX4_connection.mav.set_position_target_local_ned_send(PX4_ACC_msg.time_usec,PX4_connection.target_system,PX4_connection.target_component,1,0b0000100111111000,2.0,-2.0,-0.5,0,0,0,0,0,0,0,0) #0b101111111000
			#(END) transmitting the target command to PX4
			#----------------------------------------------------------------------------------------------------------
			#(START) arming and changing the flight mode of PX4 to OFFBOARD
			if(PX4_connection.flightmode is not 'OFFBOARD'):
				current_state_msg = PX4_connection.messages['HEARTBEAT']
				print(colored(("Current_state:%s - %u - %u" % (current_state_msg.base_mode,current_state_msg.custom_mode,current_state_msg.system_status)),"yellow"))
				num_arm = 0
				if(current_state_msg.system_status is not 4):
					Result_ARM = PX4_connection.mav.command_long_send(PX4_connection.target_system,PX4_connection.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
					num_arm = num_arm + 1
					if(num_arm>1):
						break
				if(current_state_msg.system_status is 4):
					num_mode =0
					while(PX4_connection.flightmode is not 'OFFBOARD'):
						Result_Mod_Chng = PX4_connection.mav.command_long_send(PX4_connection.target_system,PX4_connection.target_component,mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,29,6,0,0,0,0,0)
						num_mode = num_mode + 1
						if(num_mode>1):
							break
				print(colored(("PX4 Base Mode: %s - %s" % (PX4_connection.flightmode, PX4_connection.base_mode)),"yellow"))
				PX4_msg2 = PX4_connection.recv_match(type='SERVO_OUTPUT_RAW',blocking=False) #timeout=None
				print(colored(("PX4 commanded PWMs: %s" % PX4_msg2),"yellow"))
			#(END) changing the flight mode of PX4 to OFFBOARD
			#----------------------------------------------------------------------------------------------------------
			#(END) commanding to PX4
			#--------------------------------------------------------------------------------------------------------------------------


if __name__ == '__main__':
	try:
		px4_communication()
	except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
		pass


