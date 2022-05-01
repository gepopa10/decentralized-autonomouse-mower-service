import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
gpio_num_A = 26
gpio_num_B = 16
sampling_time = 10 #milliseconds
PI_ali = 3.1415
N = 20 #motor specific
GPIO.setup(gpio_num_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(gpio_num_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
while 1:
	time_init = round(time.time()*1000) #in milliseconds
	time_break = 0
	count_A_rise = 0
	count_A_fall = 0
	count_B_rise = 0
	count_B_fall = 0
	a = GPIO.input(gpio_num_A) #returns 0 or 1 for high/low
	b = GPIO.input(gpio_num_B)
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
		Delta_t = time_now - time_init #in milliseconds
		#print(Delta_t)
		if (Delta_t>1000): # we break at each 1000milliseconds so 1 sec
			time_break = 1
		if (Delta_t>sampling_time): # we break at each sampling_time so 0.01 sec
			time_break = 1
			speed_1 = (60/(2*PI_ali)) * 1000 * (2*PI_ali*count_A_rise) / (N*Delta_t) #RPM
			speed_2 = (60/(2*PI_ali)) * 1000 * (2*PI_ali*count_B_rise) / (N*Delta_t)
			count_double_A = count_A_rise + count_A_fall
			count_double_B = count_B_rise + count_B_fall
			speed_3 = (60/(2*PI_ali)) * 1000 * (2*PI_ali*count_double_A) / (2*N*Delta_t)
			speed_4 = (60/(2*PI_ali)) * 1000 * (2*PI_ali*count_double_B) / (2*N*Delta_t)
			speed_5 = (60/(2*PI_ali)) * 1000 * (2*PI_ali*(count_double_A+count_double_B)) / (4*N*Delta_t)
			print("speed-1 : %.6f" % speed_1)
			print("speed-2 : %.6f" % speed_2)
			print("speed-3 : %.6f" % speed_3)
			print("speed-4 : %.6f" % speed_4)
			print("speed-5 : %.6f" % speed_5)

#	time.sleep(1)

#GPIO.add_event_detect(23, GPIO.BOTH)
#def my_callback():
#	print("hi")
#GPIO.add_event_callback(23, my_callback)
