#!/usr/bin/env python

import rospy, tf
import serial, math
import usbdiscover
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client
from xaxxon_openlidar.cfg import XaxxonOpenLidarConfig

headeroffset = None # set by cfg
masks = [] # set by cfg
minimum_range = 0.0 # set by cfg
maximum_range = 40.0 # set by cfg 
readInterval = 0.0014 # must match firmware TODO: read from device on init
rpm = 180 # rev speed, set by cfg, 250 max
nominalcycle = 1/(rpm/60.0)
dropscan_degpersec = 0
parkoffset = 0 # used only by device firmware, set by cfg
rotateforwardoffset = 0
enable = True
ser = None
lidarrunning = False

dropScanTurnRateThreshold = 0 # 0 = disabled
lastodomth = None
lastodomtime = 0
turning = False
turnrate = 0 # radians per second

forward_offset = headeroffset
park_offset = parkoffset
read_frequency = 0
rotate_forward_offset = rotateforwardoffset
rpm_ = rpm

scan = None
rebroadcastscan = None
lastscantime = 0
BROADCASTLAST = True # continue to broadcast last scan while lidar disabled
DIR = 1 # motor direction (1=CW/RHR+ with motor @ bottom, 0=CCW, ROS default)
DEBUGOUTPUT = True

def cleanup():
	global ser, lidarrunning
	
	ser.write("f\n") # stop lidar
	lidarrunning = False
	
	if DEBUGOUTPUT:
		rospy.sleep(0.5)
		ser.reset_input_buffer()
		ser.write("z\n") # get i2c_error count
		rospy.sleep(0.1)
		while ser.inWaiting() > 0:
			line = "SCAN: "+ser.readline().strip()
			rospy.loginfo(line)
		
	ser.close()
	ser = None
	rospy.loginfo("SCAN: lidar disabled, shutdown");
	
	
def odomCallback(data):
	global turning, turnrate, lastodomth, lastodomtime

	if dropScanTurnRateThreshold == 0:
		return
		
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	odomth = tf.transformations.euler_from_quaternion(quaternion)[2] # Z rotation

	now = rospy.get_time()

	if lastodomtime == 0:
		lastodomth = odomth
		lastodomtime = now
		return
	
	turnrate = (odomth-lastodomth)/(now-lastodomtime) # radians per second

	if abs(turnrate) > dropScanTurnRateThreshold:
		turning = True
	else:
		turning = False

	lastodomth = odomth
	lastodomtime = now
	
	
def dynamicConfigCallback(config, level):
	global newheaderoffset, minimum_range, maximum_range
	global dropScanTurnRateThreshold
	global forward_offset, rpm_, park_offset, read_frequency, rotate_forward_offset
	global enable
	
	#  if DEBUGOUTPUT:
		#  print(config)
		
	updateMasks(config.masks)
	
	minimum_range = config.minimum_range
	maximum_range = config.maximum_range

	dropScanTurnRateThreshold = math.radians(config.dropscan_turnrate)
	
	forward_offset = config.forward_offset
	rpm_ = config.rpm
	park_offset = config.park_offset
	read_frequency = config.read_frequency
	rotate_forward_offset = config.rotate_forward_offset
	enable = config.lidar_enable
	
	if not ser == None:
		updateRotateForwardOffset() # needs to be before updateHeaderOffset
		updateHeaderOffset()
		updateRPM()
		updateParkOffset()
		updateReadInterval()

	return config
	
	
def updateHeaderOffset():
	global headeroffset
	
	if headeroffset == forward_offset:
		return
	
	headeroffset = forward_offset
	
	n = int(headeroffset*10)

	if DEBUGOUTPUT:
		print("sending new headeroffset to device: "+str(n))
	val1 = n & 0xFF
	val2 = (n >>8) & 0xFF
	ser.write("k")
	ser.write(chr(val1))
	ser.write(chr(val2))
	ser.write("\n")

# def getHeaderOffset():
	# global headeroffset
	
	# ser.write("i\n") # get header offset
	# rospy.sleep(0.1)
	# while ser.inWaiting() > 0:
		# line = ser.readline().strip()
		# print(line)
		# headeroffset = float(line.replace("<","").replace(">",""))*360.0
		# print("headeroffset received from device: "+str(headeroffset))
		
	# client.update_configuration({"header_offset":headeroffset})


def updateMasks(string):
	global masks
	
	#  if DEBUGOUTPUT:
		#  print("updating masks: "+string)

	masks = []
	
	try:
		strmasks = string.split()
		for s in strmasks:
			masks.append(float(s))
	except:
		rospy.logerr("error parsing masks")
		
	if not (len(masks) % 2) == 0:
		rospy.logerr("uneven number of masks")
		masks=[]
		
	updateRotateForwardOffset()


def updateRPM():
	global rpm, nominalcycle

	if rpm == rpm_:
		return
		
	rpm = rpm_
	nominalcycle = 1/(rpm/60.0)
	
	if DEBUGOUTPUT:
		print("sending rpm to device: "+str(rpm))
		
	ser.write("r"+chr(rpm)+"\n")  
	
	
def updateParkOffset():
	global parkoffset

	if parkoffset == park_offset:
		return
		
	parkoffset = park_offset
	n = int(parkoffset*10)
	
	if DEBUGOUTPUT:
		print("sending parkoffset to device: "+str(parkoffset))
		
	val1 = n & 0xFF
	val2 = (n >>8) & 0xFF
	ser.write("q")
	ser.write(chr(val1))
	ser.write(chr(val2))
	ser.write("\n")
	
	
def updateReadInterval():
	global readInterval
	
	interval = int(1.0/read_frequency*1000000)
	if interval/1000000.0 == readInterval:
		return
		
	readInterval = interval/1000000.0	
	
	if DEBUGOUTPUT:
		print("sending read interval to device: "+str(interval))
	
	val1 = interval & 0xFF
	val2 = (interval >>8) & 0xFF
	ser.write("t")
	ser.write(chr(val1))
	ser.write(chr(val2))
	ser.write("\n")


def updateRotateForwardOffset():
	global rotateforwardoffset, masks, headeroffset, forward_offset
	
	if rotateforwardoffset == rotate_forward_offset:
		return

	if DEBUGOUTPUT:
		print("setting rotate_forward_offset: "+str(rotate_forward_offset))
		
	i = 0	
	while i < len(masks):
		# un-apply old rotate_forward_offset
		masks[i] += rotateforwardoffset 
		masks[i+1] += rotateforwardoffset 
		if masks[i] >= 360 or masks[i+1] >=360:
			masks[i] -=360
			masks[i+1] -= 360
		
		# apply new rotate_forward_offset
		masks[i] -= rotate_forward_offset  
		masks[i+1] -= rotate_forward_offset
		if masks[i] < 0 or masks[i+1] < 0:
			masks[i] += 360
			masks[i+1] += 360
		
		i += 2
		
	# un-apply old rotate_forward_offset
	forward_offset -= rotateforwardoffset
	if forward_offset < 0:
		forward_offset += 360
		
	# apply new rotate_forward_offset
	forward_offset += rotate_forward_offset
	if forward_offset >= 360:
		forward_offset -= 360 
	
	rotateforwardoffset = rotate_forward_offset

	
def readlidar(ser):

	global lidarrunning, scan, lastscantime, rebroadcastscan

	lidarrunning = True	
	#  ser.write("r"+chr(rpm)+"\n")  
	ser.write("a\n") # start lidar
	rospy.loginfo("SCAN: lidar enabled");

	# clear buffer
	ser.reset_input_buffer()

	raw_data = []
	lastscan = rospy.Time.now()
	headercodesize = 4
	current_time = 0
	dropscan = False
	scannum = 0
	lastcount = 0
	consecutivescandropped = 0
	
	while not rospy.is_shutdown() and ser.is_open and enable:

		# read data and dump into array, checking for header code 0xFF,0xFF,0xFF,0xFF
		try:
			ch = ser.read(1)  
		except:
			pass
		
		# if timed out after TIMEOUT seconds, defined in usbdiscover.py)
		if len(ch) == 0:
			if scannum > 1:
				rospy.logerr("no response from xaxxonlidar device")
				scannum += 1	
				break
			continue
		
		raw_data.append(ch)
		
		if turning:
			dropscan = True
			
		if rebroadcastscan and BROADCASTLAST:
			rebroadcast()
				
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
		
		""" read cycle """
		c1 = ord(ser.read(1))
		c2 = ord(ser.read(1))
		c3 = ord(ser.read(1))
		c4 = ord(ser.read(1))
		cycle = ((c4<<24)|(c3<<16)|(c2<<8)|c1)/1000000.0

		""" host time """
		current_time = rospy.Time.now() # - rospy.Duration(0.0) #0.015
		rospycycle = current_time - lastscan
		# cycle = rospycycle.to_sec()
		lastscan = current_time	
		
		rospycount = (len(raw_data)-headercodesize)/2

		scannum += 1	


		# error monitoring, debug info
		if DEBUGOUTPUT and scannum % 10 == 0:
			msg = "SCAN "+str(scannum)+": cycle: "+str(cycle)+", count: "+str(count)
			rospy.loginfo(msg)
		
		if not rospycount == count and scannum > 1 and DEBUGOUTPUT:
			msg = "SCAN "+str(scannum)+": count/data mismatch: "+ str( rospycount-count )
			rospy.logerr(msg)

		
		if abs(count - lastcount) > 2 and scannum > 4 and DEBUGOUTPUT:
			msg = "SCAN "+str(scannum)+": count change from: "+ str(lastcount)+", to: "+str(count)
			rospy.logerr(msg)
			
		lastcount = count

		if count<5 and scannum > 4: 
			if DEBUGOUTPUT:  
				msg = "SCAN "+str(scannum)+": low data count: "+str(count)+", cycle: "+str(cycle)
				msg += ", scan dropped"
				rospy.logerr(msg)
			del raw_data[:]
			ser.write("n\n0\n") # stop broadcast, lidar disable
			rospy.sleep(0.1)
			ser.write("1\nb\n") # lidar enble, start broadcast
			consecutivescandropped += 1
			if consecutivescandropped > 3:
				rospy.logerr("resetting device")
				cleanup()
				break  # TODO: experimental
			
			continue
		else:
			consecutivescandropped = 0

		
		if scannum <= 4: # drop 1st few scans while lidar spins up
			del raw_data[:]
			continue
			
			
		rebroadcastscan = None
		
		scan = LaserScan()
		
		scan.header.stamp = current_time - rospy.Duration(cycle) # rospycycle 
		lastscantime = current_time.to_sec() - cycle # seconds, floating point (used by rebroadcastscan only)
		
		scan.header.frame_id = frameID

		scan.angle_min = 0.0
		rpmcycle = cycle
		scan.angle_max = (readInterval*(count-1))/rpmcycle * 2 * math.pi
		scan.angle_increment = (scan.angle_max-scan.angle_min) / (count-1)  	

		scan.time_increment =  readInterval
		scan.scan_time = readInterval * count # cycle # rospycycle.to_sec()

		scan.range_min = minimum_range
		scan.range_max = maximum_range

		zeroes = 0
		scan.ranges=[]
		for x in range(len(raw_data)-(count*2)-headercodesize, len(raw_data)-headercodesize, 2):
			low = ord(raw_data[x])
			high = ord(raw_data[x+1])
			value = ((high<<8)|low) / 100.0
			scan.ranges.append(value)

		""" blank frame masks W/RHR+ degrees start, stop """
		i = 0
		while i < len(masks):
			for x in range(int(count*((masks[i])/360.0)), int(count*((masks[i+1])/360.0)) ):
				scan.ranges[x] = 0
			i += 2
			
		""" if blanking scans when turning """
		if dropscan: 	
			for i in range(len(scan.ranges)):
				scan.ranges[i] = 0
		dropscan = False
		
		""" publish scan """	
		scan_pub.publish(scan)
		
		del raw_data[:] 

def rebroadcast():
	global lastscantime, rebroadcastscan
	
	if lastscantime <= (rospy.get_time() - nominalcycle*2):
		lastscantime += nominalcycle
		rebroadcastscan.header.stamp = rospy.Time.from_sec(lastscantime)
		scan_pub.publish(rebroadcastscan)


# main

rospy.init_node('lidarbroadcast', anonymous=False)
rospy.on_shutdown(cleanup)
scan_pub = rospy.Publisher(rospy.get_param('~scan_topic', 'scan'), LaserScan, queue_size=3)
rospy.Subscriber("odom", Odometry, odomCallback) # TODO: make optional?
frameID = rospy.get_param('~frame_id', 'laser_frame')

Server(XaxxonOpenLidarConfig, dynamicConfigCallback)
client = dynamic_reconfigure.client.Client("lidarbroadcast", timeout=30)
device = usbdiscover

while True:
	if ser == None and not rospy.is_shutdown(): #connect
		ser = device.usbdiscover("<id::xaxxonopenlidar>", 5)
		
		ser.write("y\n") # get version
		rospy.sleep(0.1)
		while ser.inWaiting() > 0:
			line = ser.readline().strip()
			line = "firmware: "+line
			rospy.loginfo(line)
		ser.write("d"+chr(DIR)+"\n")  # set direction (1=CW RHR+ motor@bottom default, ROS default)

	if enable:
		updateRotateForwardOffset() # needs to be 1st
		updateHeaderOffset()
		updateRPM()
		updateParkOffset()
		updateReadInterval()
		#  rebroadcastscan = None
		
		readlidar(ser) # blocking
		
	else:
		if lidarrunning:
			ser.write("f\n") # stop lidar
			lidarrunning = False
			rospy.loginfo("SCAN: lidar disabled");
			rebroadcastscan = scan	
		elif rebroadcastscan and BROADCASTLAST:
			rebroadcast()
			
		
	if rospy.is_shutdown():
		break
	else:
		device.removelockfile()
		
	rospy.sleep(0.01)

