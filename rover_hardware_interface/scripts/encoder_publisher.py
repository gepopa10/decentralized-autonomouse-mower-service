#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time
from math import pi
import scipy.signal

def encoderPublisher():
    # Load params
    frequency = rospy.get_param("encoder_read_frequency")
    gpio_num_A = rospy.get_param("encoder_gpio_A_num")
    gpio_num_B = rospy.get_param("encoder_gpio_B_num")
    N = rospy.get_param("motor_N") # motor specific

    pub = rospy.Publisher("encoder_gpio_topic", Float32, queue_size=10)
    pub_filtered = rospy.Publisher("encoder_gpio_topic_filtered", Float32, queue_size=10)
    rospy.init_node('encoder_raw_gpio', anonymous=True)
    rate = rospy.Rate(frequency)

    GPIO.setmode(GPIO.BCM)
    sampling_time = 1.0/frequency*1000 #milliseconds
    GPIO.setup(gpio_num_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(gpio_num_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    speed_1 = 0
    speed_2 = 0
    speed_3 = 0
    speed_4 = 0
    speed_5 = 0

    a_new = 0
    b_new = 0

    a = 0
    b = 0

    window_size = 11 # window size for filtering, must be odd
    polynomial_order_filter = 1 # polynomial order for filtering, works best with 1
    speeds_for_filtering = []

    while not rospy.is_shutdown():
        time_init = round(time.time()*1000) #in milliseconds
    	time_break = 0
    	count_A_rise = 0
    	count_A_fall = 0
    	count_B_rise = 0
    	count_B_fall = 0
    	a = GPIO.input(gpio_num_A) #returns 0 or 1 for high/low
    	b = GPIO.input(gpio_num_B)

        # should run 10 times for example if sampling_time = 10
    	while time_break == 0:
    		a_new = GPIO.input(gpio_num_A)
    		if ((a_new - a) == 1): # went from low to high
    			count_A_rise = count_A_rise + 1
    		if ((a_new - a) == -1): # went from high to low
    			count_A_fall = count_A_fall + 1
    		a = a_new
    		#print("Info - A: %d - %d - %d" % (a,count_A_rise,count_A_fall))
    		b_new = GPIO.input(gpio_num_B)
    		if ((b_new - b) == 1):
    			count_B_rise = count_B_rise + 1
    		if ((b_new - b) == -1):
    			count_B_fall = count_B_fall + 1
    		b = b_new
    		#print("Info - B: %d - %d - %d" % (b,count_B_rise,count_B_fall))
    		time_now = round(time.time()*1000)
                #print('time_now',time_now)
    		Delta_t = time_now - time_init #in milliseconds
    		#print(Delta_t,'Delta_t')

    		if (Delta_t>sampling_time): # we break at each sampling_time so 0.01 sec
    			time_break = 1
    			speed_1 = (60/(2*pi)) * 1000 * (2*pi*count_A_rise) / (N*Delta_t) #RPM
    			speed_2 = (60/(2*pi)) * 1000 * (2*pi*count_B_rise) / (N*Delta_t)
    			count_double_A = count_A_rise + count_A_fall
    			count_double_B = count_B_rise + count_B_fall
    			speed_3 = (60/(2*pi)) * 1000 * (2*pi*count_double_A) / (2*N*Delta_t)
    			speed_4 = (60/(2*pi)) * 1000 * (2*pi*count_double_B) / (2*N*Delta_t)
    			speed_5 = (60/(2*pi)) * 1000 * (2*pi*(count_double_A+count_double_B)) / (4*N*Delta_t)
    			# print("speed-1 : %.6f" % speed_1)
    			# print("speed-2 : %.6f" % speed_2)
    			# print("speed-3 : %.6f" % speed_3)
    			# print("speed-4 : %.6f" % speed_4)
    			# print("speed-5 : %.6f" % speed_5)

        speeds_for_filtering.append(speed_5)
        speed_5_filtered = speed_5
        #print('len(speeds_for_filtering)',len(speeds_for_filtering))
        if (len(speeds_for_filtering) >= window_size):
            speeds_5_filtered = scipy.signal.savgol_filter(speeds_for_filtering, window_size, polynomial_order_filter)
            speed_5_filtered = speeds_5_filtered[-1]
            speeds_for_filtering=speeds_for_filtering[1::] #remove the first value

	pub.publish(speed_5) # we publish the speed calculate with the 5th method
	pub_filtered.publish(speed_5_filtered)

	rate.sleep()



if __name__ == '__main__':
    try:
        encoderPublisher()
    except rospy.ROSInterruptException:
        pass
