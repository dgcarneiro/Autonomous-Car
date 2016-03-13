import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import time
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import serial
import Adafruit_BBIO.UART as UART
from math import *
import smbus
import Adafruit_BBIO.GPIO as GPIO
import thread
import threading
import math

#Compass setup
bus = smbus.SMBus(1)
address = 0x1e

#GPS setup
UART.setup("UART4")
UART.setup("UART2")
UART.setup("UART5")
gps1 = serial.Serial("/dev/ttyO4", 4800)
gps2 = serial.Serial("/dev/ttyO2", 4800)
gps3 = serial.Serial("/dev/ttyO5", 4800)

hlock = thread.allocate_lock() ##This was added by john to test compass ...

#giving value of 0 to variables
line = 0
kp = .5
avoid_flag = 0
bearing = 0
distance =0
fix_flag = 0
heading = 0
dest_lat=dest_long=0
avg_lat = avg_long = 0
latitude1 = latitude2 = latitude3 = 0
longitude1 = longitude2 = longitude3 = 0


# Defined GPIO pins for sonar
GPIO_TRIGGER = "P8_7"
GPIO_ECHO    = "P8_8"

# Set pins as output and input
GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO,GPIO.IN)      # Echo

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGGER, GPIO.LOW)


# Right side motors. Pin definitions for H-bridge control
right_wheel_speed = "P9_14"  # controls speed of right front motor via PWM
right_wheel_direction = "P9_12"   # combine M1 pins on upper and lower motor controllers into a single pin

# Left side motors. Pin definitions for H-bridge control
left_wheel_speed = "P9_16"  # controls speed of left front motor via PWM
left_wheel_direction = "P8_11"  # combine M2 pins on upper and lower motor controllers into a single pin

# set pin directions as output
GPIO.setup(right_wheel_speed, GPIO.OUT)
GPIO.setup(right_wheel_direction, GPIO.OUT)

GPIO.setup(left_wheel_speed, GPIO.OUT)
GPIO.setup(left_wheel_direction, GPIO.OUT)

PWM.start(right_wheel_speed, 0)
PWM.start(left_wheel_speed, 0)


def measure():
	global avoid_flag
	# This function measures a distance
	GPIO.output(GPIO_TRIGGER, GPIO.HIGH)
	time.sleep(0.00001)
	GPIO.output(GPIO_TRIGGER, GPIO.LOW)
	start = time.time()

	while GPIO.input(GPIO_ECHO)==0:
		start = time.time()

	while GPIO.input(GPIO_ECHO)==1:
		stop = time.time()

	elapsed = stop-start
	distance = (elapsed * 34300)/2
	if distance < 20:
		avoid_flag = 1
	else:
		avoid_flag = 0
		
def fix_check():
	global fix_flag
	gps1.open()
	data1 = gps1.readline()
	gps1.close()
	data_array1 = data1.split(',')
	
	gps2.open()
	data2 = gps2.readline()
	gps2.close()
	data_array2 = data2.split(',')
	
	gps3.open()
	data3 = gps3.readline()
	gps3.close()
	data_array3 = data3.split(',')
	
	if len(data1) < 50 or len(data2) < 50 or len(data3) < 50:
		fix_flag =0
	else:
		fix_flag = 1
	
	#print fix_flag
		
def get_location():

	global avg_lat
	global avg_long
	
	gps1.open()
	data1 = gps1.readline()
	gps1.close()
	data_array1 = data1.split(',')
	
	gps2.open()
	data2 = gps2.readline()
	gps2.close()
	data_array2 = data2.split(',')
	
	gps3.open()
	data3 = gps3.readline()
	gps3.close()
	data_array3 = data3.split(',')
	
	if data_array1[0]=='$GPGGA':
		latitude1 = float(data_array1[2])
		longitude1 = float(data_array1[4])

		number_lat1 = (str(latitude1))[2:]
		number_lat1 = float(number_lat1)/60
		number_lat1 = float(int(latitude1*(.01))+number_lat1)
		latitude1 = number_lat1
		
		number_lon1 = (str(longitude1))[2:]
		number_lon1 = float(number_lon1)/60
		number_lon1 = float(int(longitude1*10**-2)+number_lon1)
		longitude1 = number_lon1
					
	if data_array2[0]=='$GPGGA':
		latitude2= float(data_array2[2])
		longitude2 = float(data_array2[4])
		
		number_lat2 = (str(latitude2))[2:]
		number_lat2 = float(number_lat2)/60
		number_lat2 = float(int(latitude2*10**-2)+number_lat2)
		latitude2 = number_lat2
		
		number_lon2 = (str(longitude2))[2:]
		number_lon2 = float(number_lon2)/60
		number_lon2 = float(int(longitude2*10**-2)+number_lon2)
		longitude2 = number_lon2
		
	if data_array3[0]=='$GPGGA':
		latitude3= float(data_array3[2])
		longitude3 = float(data_array3[4])
		
		number_lat3 = (str(latitude3))[2:]
		number_lat3 = float(number_lat3)/60
		number_lat3 = float(int(latitude3*10**-2)+number_lat3)
		latitude3 = number_lat3
		
		number_lon3 = (str(longitude3))[2:]
		number_lon3 = float(number_lon3)/60
		number_lon3 = float(int(longitude3*10**-2)+number_lon3)
		longitude3 = number_lon3		
		
	avg_lat = (float(latitude1) + float(latitude2)+ float(latitude3))/3 
	avg_long = -1*(float(longitude1) + float(longitude2)+float(longitude3))/3
		
def readfile_coordinates():
	global dest_lat
	global dest_long
	file = open('waypoints.txt', 'r')
	locations = file.readlines()
	directions = []
	for location in locations:
		location_list = location.split(',')
		location_float = []
		for val in location_list:
			location_float.append(float(val))
		directions.append(location_float)
	file.close()
	dest_lat = directions[line][0]
	dest_long = directions[line][1]

def cruise():
	GPIO.output(right_wheel_direction, GPIO.HIGH)
	GPIO.output(left_wheel_direction, GPIO.HIGH)
	PWM.set_duty_cycle(right_wheel_speed, 55-gain)
	PWM.set_duty_cycle(left_wheel_speed, 50+gain)

def forward():
	GPIO.output(right_wheel_direction, GPIO.HIGH)
	GPIO.output(left_wheel_direction, GPIO.HIGH)

def backward():
	GPIO.output(right_wheel_direction, GPIO.LOW)
	GPIO.output(left_wheel_direction, GPIO.LOW)

def spin_left():
	GPIO.output(right_wheel_direction, GPIO.HIGH)
	GPIO.output(left_wheel_direction, GPIO.LOW)

def spin_right():
	GPIO.output(right_wheel_direction, GPIO.LOW)
	GPIO.output(left_wheel_direction, GPIO.HIGH)

def speed():
	PWM.set_duty_cycle(right_wheel_speed, 55)
	PWM.set_duty_cycle(left_wheel_speed, 55)

def stop_motor():
	PWM.set_duty_cycle(right_wheel_speed, 0)
	PWM.set_duty_cycle(left_wheel_speed, 0)
	
def distance1(avg_lat,avg_long,dest_lat,dest_long):
	
	global distance
	#convert to radians
	avg_long, avg_lat, dest_long, dest_lat = map(radians, [float(avg_long), float(avg_lat), float(dest_long), float(dest_lat)])
	
	#distance calculation probably not needed
	dlon = dest_long - avg_long
	dlat = dest_lat - avg_lat
	a = sin(dlat/2)**2 + cos(avg_lat) * cos(dest_lat) * sin(dlon/2)**2
	#c = 2 * atan2(sqrt(a), sqrt(1-a))
	c = 2*asin(sqrt(a))
	#returns distance in cm
	distance = (6371 * c) * 100000
	

def direction(avg_long, avg_lat, dest_long, dest_lat):
	global bearing
	#convert to radians
	avg_long, avg_lat, dest_long, dest_lat = map(radians, [avg_long, avg_lat, dest_long, dest_lat])
	
	#bearing calculation
	bearing = atan2(sin(dest_long-avg_long)*cos(dest_lat), cos(avg_lat)*sin(dest_lat)-sin(avg_lat)*cos(dest_lat)*cos(dest_long-avg_long))
	bearing = degrees(bearing)
	bearing = (bearing + 360) % 360
	
#compass def	
def read_byte(adr):
    return bus.read_byte_data(address, adr)
	
#compass def
def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val
	
#compass def
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

#compass def
def write_byte(adr, value):
    bus.write_byte_data(address, adr, value)	
	
	
def compass():
	while True:
		global heading
		write_byte(0, 0b01110000)
		write_byte(1, 0b00100000)
		write_byte(2, 0b00000000)
		
		x_offset = 9
		y_offset = -80
		x_out = (read_word_2c(3) - x_offset) * 0.92
		y_out = (read_word_2c(7) - y_offset) * 0.92
		#z_out = (read_word_2c(5)) * 0.92
		
		hlock.acquire()  ##This was added by john to test compass
		heading  = (math.atan2(y_out, x_out))
		if (heading < 0):
			heading += 2 * math.pi
			
		#declination is -10*31' = -10.517
		#http://www.rapidtables.com/convert/number/degrees-minutes-seconds-to-degrees.htm		
		heading = (math.degrees(heading))
		#print "In compass(): heading is ", heading
		hlock.release() ##This was added by john to test compass


while True:
	#thread1 = threading.Thread(target=get_location, args=())
	#thread2 = threading.Thread(target=compass, args=())	
	
	
	#while fix_flag ==0:
	#
	#fix_check()
	#thread1.start()
	#thread2.start()
	thread.start_new_thread(compass, ()) ## This was moved by john to test compass
	
	try:
		while True:
			#measure()
			#fix_check()
			#thread.start_new_thread(get_location, ())
			get_location()
			readfile_coordinates()
			#compass()
			#thread.start_new_thread(compass, ())
			#print heading
			#distance1(avg_lat,avg_long,dest_lat,dest_long)
			#print 1
			direction(avg_long, avg_lat, dest_long, dest_lat)
			#print 2
			if (avoid_flag==1):
				stop_motor()
				
			#needs tuning to give a reasonable threshold, if less than 20cm to waypoint
			#elif distance < 200:
			print dest_long
			
			if avg_long > (dest_long+ 0.5) or avg_long < (dest_long - 0.5):
				if avg_lat > (dest_lat + 0.5) or avg_lat < (dest_lat- 0.5):
					line = line+1
					
			

			
			else:
				print "the line value is ", line
				#print "the distance is ", distance
				#error calculation for kp
				hlock.acquire()  ##This was added by john to test out compass
				error = bearing - (heading-10.517)
				print "Current coordinates are ",avg_lat,avg_long
				print "Coordinates of the next waypoint is ", dest_lat,dest_long				
				print "the angle difference is ", error
				#print " "
				print "The heading is ", heading
				hlock.release() ##This was added by john to test out compass
				print "the bearing is ", bearing
				#gain calculation for kp
				gain = kp*error
				#print "the gain is ", gain
				if ((gain> 50) or gain <-30):
					gain = 30
				#cruise()
					
				#time.sleep(.001)
	except KeyboardInterrupt: # pressing ctrl C stops operation and cleans up

		PWM.stop(right_wheel_speed)	
		PWM.stop(left_wheel_speed)
		PWM.cleanup()
		GPIO.cleanup()
