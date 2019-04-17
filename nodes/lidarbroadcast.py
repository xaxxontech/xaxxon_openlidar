#!/usr/bin/env python

import rospy, tf
import serial, math
import usbdiscover
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client
from xaxxon_openlidar.cfg import XaxxonOpenLidarConfig


READINTERVAL = 0.0014 # must match firmware TODO: read from device on init
# SKIPLAST = 6 # must match firmware TODO: read from device on init
MINIMUMRANGE = 0.5 # 0.5
MAXMIMURANGE = 40.0 
RPM = 220 # speed, firmware default 180
DIR = 1 # motor direction (1=CW/RHR+ with motor @ bottom, 0=CCW, ROS default)
debugoutput = True
VERSION = "1.0"
lastodomth = None
lastodomtime = 0
turning = False
turnrate = 0 # radians per second
headeroffset = None
newheaderoffset = None
masks = [] 
masksConfigInit = False

def cleanup():
	global ser
	ser.write("f\n") # stop lidar
	ser.close()
	removelockfiles()
	rospy.sleep(3)
	print("lidar disabled, shutdown");
	
def odomCallback(data):
	global turning, turnrate, lastodomth, lastodomtime
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	odomth = tf.transformations.euler_from_quaternion(quaternion)[2] # Z rotation

	now = rospy.get_time()
	
	# print("odomth"+str(odomth))
	
	if lastodomtime == 0:
		lastodomth = odomth
		lastodomtime = now
		return
	
	turnrate = (odomth-lastodomth)/(now-lastodomtime) # radians per second

	# print(str(turnrate))
	
	if abs(turnrate) > 0.785: # 45 deg per second TODO: roslaunch parameter
		turning = True
	else:
		turning = False

	lastodomth = odomth
	lastodomtime = now
	
def dynamicConfigCallback(config, level):
	global newheaderoffset
	
	print(config)
	
	if not config.header_offset == headeroffset and not headeroffset == None:
		newheaderoffset = config.header_offset
		print ("newheaderoffset: "+ str(newheaderoffset))
		
	if masksConfigInit:	
		updateMasks(config.masks)

	return config
	
def updateHeaderOffset():
	global headeroffset, newheaderoffset

	headeroffset = newheaderoffset
	newheaderoffset = None
	
	n = int(headeroffset*10)

	print("sending newheaderoffset to device: "+str(headeroffset))
	val1 = n & 0xFF
	val2 = (n >>8) & 0xFF
	ser.write("k")
	ser.write(chr(val1))
	ser.write(chr(val2))
	ser.write("\n")

	rospy.sleep(0.1)

def getHeaderOffset():
	global headeroffset
	
	ser.write("i\n") # get header offset
	rospy.sleep(0.1)
	while ser.inWaiting() > 0:
		line = ser.readline().strip()
		print(line)
		headeroffset = float(line.replace("<","").replace(">",""))*360.0
		print("headeroffset received from device: "+str(headeroffset))
		
	client.update_configuration({"header_offset":headeroffset})

def updateMasks(string):
	global masks
	
	print("updating masks: "+string)
	
	strmasks = string.split()
	masks = []
	i=0
	for s in strmasks:
		masks[i]=int(s)
		i += 1
		
def writeMasksToConfig():
	newconfig = ""
	for i in masks:
		newconfig += str(i)+" "
	
	print("masks newconfig: "+newconfig)
	client.update_configuration({"masks":newconfig})
	
	masksConfigInit = True
		

# main

rospy.init_node('lidarbroadcast', anonymous=False)
rospy.on_shutdown(cleanup)
rospy.loginfo("xaxxon_openlidar.py version: "+VERSION)
scan_pub = rospy.Publisher(rospy.get_param('~scan_topic', 'scan'), LaserScan, queue_size=3)
rospy.Subscriber("odom", Odometry, odomCallback) # TODO: make optional
Server(XaxxonOpenLidarConfig, dynamicConfigCallback)
client = dynamic_reconfigure.client.Client("lidarbroadcast", timeout=30)

writeMasksToConfig()

ser = usbdiscover.usbdiscover("<id::xaxxonopenlidar>")

ser.write("y\n") # get version
line = ""
rospy.sleep(0.1)
while ser.inWaiting() > 0:
	line = ser.readline().strip()
	print(line)
	
getHeaderOffset()

ser.write("r"+chr(RPM)+"\n")  # set speed (180 default, 255 max) 
ser.write("d"+chr(DIR)+"\n")  # set direction (1=CW RHR+ motor@bottom default, ROS default)
ser.write
ser.write("a\n") # start lidar

# clear buffer
ser.reset_input_buffer()

raw_data = []
lastscan = rospy.Time.now()
headercodesize = 4
current_time = 0
dropscan = False
scannum = 0

while not rospy.is_shutdown() and ser.is_open:
	
	# read data and dump into array, checking for header code 0xFF,0xFF,0xFF,0xFF
	ch = ser.read(1)
	
	if len(ch) == 0:
		rospy.logerr("no response from xaxxonlidar device")
		break
	
	raw_data.append(ch)
	
	if turning:
		dropscan = True
			
	if not ord(ch) == 0xFF:
		continue
	else:
		ch = ser.read(1)
		raw_data.append(ch)
		if not ord(ch) == 0xFF:
			continue
		else: 
			ch = ser.read(1)
			raw_data.append(ch)
			if not ord(ch) == 0xFF:
				continue
			else: 
				ch = ser.read(1)
				raw_data.append(ch)
				if not ord(ch) == 0xFF:
					continue

	ser.write("h\n") # send host hearbeat (every <10 sec minimum)

	# read count		
	low = ord(ser.read(1))
	high = ord(ser.read(1))
	count = (high<<8)|low

	# """ read first distance offset """
	# low = ord(ser.read(1))
	# high = ord(ser.read(1))
	# firstDistanceOffset = ((high<<8)|low)/1000000.0
	
	""" read cycle """
	c1 = ord(ser.read(1))
	c2 = ord(ser.read(1))
	c3 = ord(ser.read(1))
	c4 = ord(ser.read(1))
	cycle = ((c4<<24)|(c3<<16)|(c2<<8)|c1)/1000000.0
	
	# """ read last distance offset """
	# low = ord(ser.read(1))
	# high = ord(ser.read(1))
	# lastDistanceOffset = ((high<<8)|low)/1000000.0
	
	# """ device time """
	# if current_time == 0:
		# current_time = rospy.Time.now() # - rospy.Duration(0.0) #0.015
	# else:
		# current_time += rospy.Duration(cycle)
	# rospycycle = current_time - lastscan
	# lastscan = current_time

	""" host time """
	current_time = rospy.Time.now() # - rospy.Duration(0.0) #0.015
	rospycycle = current_time - lastscan
	# cycle = rospycycle.to_sec()
	lastscan = current_time	

	
	rospycount = (len(raw_data)-headercodesize)/2
	
	# startup debug info
	if debugoutput:
		if not count == 0:
			print "cycle: "+str(cycle)
			## print "rospycycle: "+str(rospycycle.to_sec())
			print "count: "+str(count)
			# print "lastDistanceOffset: "+str(lastDistanceOffset)
			## print "firstDistanceOffset: "+str(firstDistanceOffset)
			print "scannum: "+str(scannum)
			# print "interval: "+str(cycle/count)
			## print "raw_data length: "+str((len(raw_data)-headercodesize)/2)
		if not rospycount == count:
			print "*** COUNT/DATA MISMATCH *** "+ str( rospycount-count )
		print " "
	
		# if scannum > 10:
			# debugoutput = False
	
	scannum += 1	
	if scannum <= 3: # drop 1st few scans while lidar spins up
		del raw_data[:]
		continue
	
	scan = LaserScan()
	scan.header.stamp = current_time - rospy.Duration(cycle) # rospycycle 
	scan.header.frame_id = 'laser_frame'

	# scan.angle_min = 0.0
	# scan.angle_max = (cycle - READINTERVAL*SKIPLAST)/cycle * 2 * math.pi
	# scan.angle_increment = (scan.angle_max-scan.angle_min) / (count-1)  	


	scan.angle_min = 0.0
	# scan.angle_max = (READINTERVAL*(count-1))/cycle * 2 * math.pi
	rpmcycle = 1.0/(RPM/60.0)
	scan.angle_max = (READINTERVAL*(count-1))/rpmcycle * 2 * math.pi
	scan.angle_increment = (scan.angle_max-scan.angle_min) / (count-1)  	

	""" distort scan to comp for rotating mobile base """
	# if turnrate>0: # radians per second, robot turning CW, spread out scan		
		# ratio = 1+turnrate/(2*math.pi/cycle)
		# scan.angle_max *= ratio
		# scan.angle_increment *= ratio
	# elif turnrate<0: # robot turning CCW, compress scan
		# ratio = 1-turnrate/(2*math.pi/cycle)
		# scan.angle_max *= ratio
		# scan.angle_increment *= ratio

	# scan.time_increment =  cycle/count
	# scan.scan_time = cycle # rospycycle.to_sec()

	scan.time_increment =  READINTERVAL
	scan.scan_time = READINTERVAL * count # cycle # rospycycle.to_sec()

	scan.range_min = MINIMUMRANGE
	scan.range_max = MAXMIMURANGE

	zeroes = 0
	scan.ranges=[]
	for x in range(len(raw_data)-(count*2)-headercodesize, len(raw_data)-headercodesize, 2):
		low = ord(raw_data[x])
		high = ord(raw_data[x+1])
		value = ((high<<8)|low) / 100.0
		scan.ranges.append(value)

	""" comp rpm photo sensor offset """
	# tilt = 283 # degrees  (was 280)
	# split = int(tilt/360.0*count)
	# scan.ranges = temp[split:]+temp[0:split]

	""" blank frame masks W/RHR+ degrees start, stop """
	i = 0
	while i < len(masks):
		for x in range(int(count*((masks[i])/360.0)), int(count*((masks[i+1])/360.0)) ):
			scan.ranges[x] = 0
		i += 2
		
		
	""" blank scans when turning """
	if dropscan: 	
		for i in range(len(scan.ranges)):
			scan.ranges[i] = 0
	dropscan = False
	
	""" publish scan """	
	scan_pub.publish(scan)
	
	del raw_data[:] 
	
	if not newheaderoffset == None:
		updateHeaderOffset()

	# if scannum % 10 == 0:
		# msg = "scan #: "+str(scannum)
		# rospy.loginfo(msg)

