
import rclpy 
import math, time
from array import *
import xaxxon_openlidar.usbdiscover as device
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterEvent

# import signal, sys

#parameter defaults
rpm=180
minimum_range = 0.5
maximum_range = 40.0 
masks = [80.0, 97.0, 263.0, 280.0]
dropscan_turnrate = 0 # threshold, degrees, 0 = disabled
park_offset = 180.0 # used only by device firmware
forward_offset = 189.0
read_frequency = 715.0
rotate_forward_offset = 0.0
lidar_enable = True
scan_topic = "scan"
frame_id = "laser_frame"

# vars
node = None  # node object
scan_pub = None # publisher object
ser = None   # serial object
lidarrunning = False
readInterval = 0.0 

lastodomth = None
lastodomtime = 0
turning = False
turnrate = 0 # radians per second

scan = None
rebroadcastscan = None
lastscantime = 0

oldrotateforwardoffset = 0
nominalcycle = 0
shuttingdown = False

#constants
BROADCASTLAST = True # continue to broadcast last scan while lidar disabled
DIR = 1 # motor direction (1=CW/RHR+ with motor @ bottom, 0=CCW, ROS default)
DEBUGOUTPUT = True


def cleanup():
	global ser, lidarrunning, shuttingdown
	
	if shuttingdown:
		return
	
	shuttingdown = True
	
	ser.write(b"f\n") # stop lidar
	lidarrunning = False
	
	if DEBUGOUTPUT:
		print("cleanup")
		time.sleep(0.5)
		ser.reset_input_buffer()
		ser.write(b"z\n") # get i2c_error count
		time.sleep(0.1)
		while ser.inWaiting() > 0:
			line = "SCAN: "+ser.readline().decode("utf-8").strip()
			node.get_logger().info(line)
		
	ser.close()
	ser = None
	node.get_logger().info("SCAN: lidar disabled, shutdown");
	rclpy.shutdown()

	
def odomCallback(data):
	global turning, turnrate, lastodomth, lastodomtime

	if dropscan_turnrate == 0:
		return
		
	quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
	data.pose.pose.orientation.z, data.pose.pose.orientation.w )
	odomth = tf.transformations.euler_from_quaternion(quaternion)[2] # Z rotation

	now = rclpy.get_time()

	if lastodomtime == 0:
		lastodomth = odomth
		lastodomtime = now
		return
	
	turnrate = (odomth-lastodomth)/(now-lastodomtime) # radians per second

	if abs(turnrate) > math.radians(dropscan_turnrate):
		turning = True
	else:
		turning = False

	lastodomth = odomth
	lastodomtime = now
	
	
def declareParams():
	global node
	global minimum_range, maximum_range, rpm, masks, dropscan_turnrate, park_offset
	global forward_offset, read_frequency, rotate_forward_offset, lidar_enable, frame_id, scan_topic
	
	node.declare_parameter("minimum_range")
	node.declare_parameter("maximum_range")
	node.declare_parameter("rpm")
	node.declare_parameter("masks")
	node.declare_parameter("dropscan_turnrate")
	node.declare_parameter("park_offset")
	node.declare_parameter("forward_offset")
	node.declare_parameter("read_frequency")
	node.declare_parameter("rotate_forward_offset")
	node.declare_parameter("lidar_enable")
	node.declare_parameter("frame_id")
	node.declare_parameter("scan_topic")

	# if not node.get_parameter("minimum_range").type_.value == 0:
	if not str(node.get_parameter("minimum_range").type_) == "Type.NOT_SET":
		minimum_range = node.get_parameter("minimum_range").get_parameter_value().double_value
		print("setting minimum_range: "+str(minimum_range))
		
	if not str(node.get_parameter("maximum_range").type_) == "Type.NOT_SET":
		maximum_range = node.get_parameter("maximum_range").get_parameter_value().double_value
		print("setting maximum_range: "+str(maximum_range))
	
	if not str(node.get_parameter("rpm").type_) == "Type.NOT_SET":
		rpm = node.get_parameter("rpm").get_parameter_value().integer_value
		print("setting rpm: "+str(rpm))
		
	if not str(node.get_parameter("masks").type_) == "Type.NOT_SET":
		masks = node.get_parameter("masks").get_parameter_value().double_array_value
		print("setting masks: "+str(masks))

	if not str(node.get_parameter("dropscan_turnrate").type_) == "Type.NOT_SET":
		dropscan_turnrate = node.get_parameter("dropscan_turnrate").get_parameter_value().integer_value
		print("setting dropscan_turnrate: "+str(dropscan_turnrate))
		
	if not str(node.get_parameter("park_offset").type_) == "Type.NOT_SET":
		park_offset = node.get_parameter("park_offset").get_parameter_value().double_value
		print("setting park_offset: "+str(park_offset))
		
	if not str(node.get_parameter("forward_offset").type_) == "Type.NOT_SET":
		forward_offset = node.get_parameter("forward_offset").get_parameter_value().double_value
		print("setting forward_offset: "+str(forward_offset))		
		
	if not str(node.get_parameter("read_frequency").type_) == "Type.NOT_SET":
		read_frequency = node.get_parameter("read_frequency").get_parameter_value().double_value
		print("setting read_frequency: "+str(read_frequency))
		
	if not str(node.get_parameter("rotate_forward_offset").type_) == "Type.NOT_SET":
		rotate_forward_offset = node.get_parameter("rotate_forward_offset").get_parameter_value().double_value
		print("setting rotate_forward_offset: "+str(rotate_forward_offset))
		
	if not str(node.get_parameter("lidar_enable").type_) == "Type.NOT_SET":
		lidar_enable = node.get_parameter("lidar_enable").get_parameter_value().bool_value
		print("setting lidar_enable: "+str(lidar_enable))			
		
	if not str(node.get_parameter("frame_id").type_) == "Type.NOT_SET":
		frame_id = node.get_parameter("frame_id").get_parameter_value().string_value
		print("setting frame_id: "+str(frame_id))
		
	if not str(node.get_parameter("scan_topic").type_) == "Type.NOT_SET":
		scan_topic = node.get_parameter("scan_topic").get_parameter_value().string_value
		print("setting scan_topic: "+str(scan_topic))	
					
	"""
	all_new_parameters = [
		rclpy.parameter.Parameter(
			"minimum_range",
			rclpy.Parameter.Type.DOUBLE,
			minimum_range
		),
		rclpy.parameter.Parameter(
			"maximum_range",
			rclpy.Parameter.Type.DOUBLE,
			maximum_range
		),
		rclpy.parameter.Parameter(
			"rpm",
			rclpy.Parameter.Type.INTEGER,
			rpm
		),
		rclpy.parameter.Parameter(
			"masks",
			rclpy.Parameter.Type.DOUBLE_ARRAY,
			masks
		),
		rclpy.parameter.Parameter(
			"dropscan_turnrate",
			rclpy.Parameter.Type.INTEGER,
			dropscan_turnrate
		),
		rclpy.parameter.Parameter(
			"park_offset",
			rclpy.Parameter.Type.DOUBLE,
			park_offset
		),
			rclpy.parameter.Parameter(
			"forward_offset",
			rclpy.Parameter.Type.DOUBLE,
			forward_offset
		),
		rclpy.parameter.Parameter(
			"read_frequency",
			rclpy.Parameter.Type.DOUBLE,
			read_frequency
		),
		rclpy.parameter.Parameter(
			"rotate_forward_offset",
			rclpy.Parameter.Type.DOUBLE,
			rotate_forward_offset
		),
		rclpy.parameter.Parameter(
			"lidar_enable",
			rclpy.Parameter.Type.BOOL,
			lidar_enable
		),
		rclpy.parameter.Parameter(
			"frame_id",
			rclpy.Parameter.Type.STRING,
			frame_id
		),
		rclpy.parameter.Parameter(
			"scan_topic",
			rclpy.Parameter.Type.STRING,
			scan_topic
		)
	]

	node.set_parameters(all_new_parameters)
	"""


def parameterscallback(msg):
	global minimum_range, maximum_range, rpm, masks, dropscan_turnrate, park_offset
	global forward_offset, read_frequency, rotate_forward_offset, lidar_enable, frame_id, scan_topic

	print("param event happened")
	print(msg)

	if msg.node=="/lidarbroadcast":
		params = msg.new_parameters + msg.changed_parameters
		for param in params:
			
			if param.name == "minimum_range":
				minimum_range = node.get_parameter(param.name).get_parameter_value().double_value
				print("minimum_range changed to: "+str(minimum_range))
			
			elif param.name == "maximum_range":
				maximum_range = node.get_parameter(param.name).get_parameter_value().double_value
				print("maximum_range changed to: "+str(maximum_range))
				
			elif param.name == "rpm":
				rpm = node.get_parameter(param.name).get_parameter_value().integer_value
				print("rpm changed to: "+str(rpm))
				updateRPM()

			elif param.name == "masks":
				masks = node.get_parameter(param.name).get_parameter_value().double_array_value
				print("masks changed to: "+str(masks))
				updateMasks()
				
			elif param.name == "dropscan_turnrate":
				dropscan_turnrate = node.get_parameter(param.name).get_parameter_value().integer_value
				print("dropscan_turnrate changed to: "+str(dropscan_turnrate))

			elif param.name == "park_offset":
				park_offset = node.get_parameter(param.name).get_parameter_value().integer_value
				print("park_offset changed to: "+str(park_offset))
				updateParkOffset()
				
			elif param.name == "forward_offset":
				forward_offset = node.get_parameter(param.name).get_parameter_value().double_value
				print("forward_offset changed to: "+str(forward_offset))
				updateRotateForwardOffset() # needs to be before updateForwardOffset
				updateForwardOffset()
				
			elif param.name == "read_frequency":
				read_frequency = node.get_parameter(param.name).get_parameter_value().double_value
				print("read_frequency changed to: "+str(read_frequency))
				updateReadInterval()
				
			elif param.name == "rotate_forward_offset":
				rotate_forward_offset = node.get_parameter(param.name).get_parameter_value().double_value
				print("rotate_forward_offset changed to: "+str(rotate_forward_offset))
				updateRotateForwardOffset() # needs to be before updateForwardOffset
				updateForwardOffset()
				
			elif param.name == "lidar_enable":
				lidar_enable = node.get_parameter(param.name).get_parameter_value().bool_value
				print("lidar_enable changed to: "+str(lidar_enable))

			elif param.name == "frame_id":
				frame_id = node.get_parameter(param.name).get_parameter_value().string_value
				print("frame_id changed to: "+frame_id)
				
			elif param.name == "scan_topic":
				scan_topic = node.get_parameter(param.name).get_parameter_value().string_value
				print("scan_topic changed to: "+scan_topic)		
						
	
def updateForwardOffset():
	n = int(forward_offset*10)

	if DEBUGOUTPUT:
		print("sending new foward offset to device: "+str(n))
	val1 = n & 0xFF
	val2 = (n >>8) & 0xFF
	# ser.write(("k" + chr(val1) + chr(val2) + "\n").encode())
	# ser.write(b"k")
	# ser.write(val1)
	# ser.write(val2)
	# ser.write(b"\n")
	ser.write(bytearray([ord("k"), val1, val2, ord("\n")]))


def updateMasks():
	if not (len(masks) % 2) == 0:
		node.get_logger().error("uneven number of masks")
		masks=[]
		
	updateRotateForwardOffset()


def updateRPM():
	global nominalcycle

	nominalcycle = 1/(rpm/60.0)
	
	if DEBUGOUTPUT:
		print("sending rpm to device: "+str(rpm))
		
	# ser.write(("r"+chr(rpm)+"\n").encode())
	
	# ser.write(b"r")
	# ser.write(rpm)
	# ser.write(b"\n")
	
	# ser.write(bytearray([b"r", rpm, b"\n"]))
	
	# ser.write(b"r"+chr(rpm)+b"\n")
	
	ser.write(bytearray([ord("r"), rpm, ord("\n")]))
	
	
def updateParkOffset():
	n = int(park_offset*10)
	
	if DEBUGOUTPUT:
		print("sending parkoffset to device: "+str(park_offset))
		
	val1 = n & 0xFF
	val2 = (n >>8) & 0xFF
	# ser.write(("q"+chr(val1)+chr(val2)+"\n").encode())
	# ser.write(b"q")
	# ser.write(val1)
	# ser.write(val2)
	# ser.write(b"\n")
	ser.write(bytearray([ord("q"), val1, val2, ord("\n")]))

	
def updateReadInterval():
	global readInterval
	
	readInterval = 1.0/read_frequency # must match firmware TODO: read from device on init

	interval = int(1.0/read_frequency*1000000)
	
	if DEBUGOUTPUT:
		print("sending read interval to device: "+str(interval))
	
	val1 = interval & 0xFF
	val2 = (interval >>8) & 0xFF
	# ser.write(("t"+chr(val1)+chr(val2)+"\n").encode())
	# ser.write(b"t")
	# ser.write(val1)
	# ser.write(val2)
	# ser.write(b"\n")
	ser.write(bytearray([ord("t"), val1, val2, ord("\n")]))


def updateRotateForwardOffset():
	global masks, forward_offset, oldrotateforwardoffset

	if DEBUGOUTPUT:
		print("setting rotate_forward_offset: "+str(rotate_forward_offset))
		
	i = 0	
	while i < len(masks):
		# un-apply old rotate_forward_offset
		masks[i] += oldrotateforwardoffset 
		masks[i+1] += oldrotateforwardoffset 
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
	forward_offset -= oldrotateforwardoffset
	if forward_offset < 0:
		forward_offset += 360
		
	# apply new rotate_forward_offset
	forward_offset += rotate_forward_offset
	if forward_offset >= 360:
		forward_offset -= 360 
	
	oldrotateforwardoffset = rotate_forward_offset

	
def readlidar(ser):
	global lidarrunning, scan, lastscantime, rebroadcastscan, node

	lidarrunning = True	
	#  ser.write("r"+chr(rpm)+"\n")  
	ser.write(b"a\n") # start lidar
	node.get_logger().info("SCAN: lidar enabled");

	# clear buffer
	ser.reset_input_buffer()

	raw_data = []
	lastscan = node.get_clock().now()
	headercodesize = 4
	current_time = 0
	dropscan = False
	scannum = 0
	lastcount = 0
	consecutivescandropped = 0
	
	while ser.is_open and lidar_enable and rclpy.ok():

		# read data and dump into array, checking for header code 0xFF,0xFF,0xFF,0xFF
		try:
			ch = ser.read(1)  
		except KeyboardInterrupt: # ctrl-c pressed
			cleanup()
		
		# if timed out after TIMEOUT seconds, defined in usbdiscover.py)
		if len(ch) == 0:
			if scannum > 1:
				node.get_logger().error("no response from xaxxonlidar device")
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

		ser.write(b"h\n") # send host hearbeat (every <10 sec minimum)

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
		current_time = node.get_clock().now() # - rclpy.Duration(0.0) #0.015
		rclpycycle = current_time - lastscan
		# cycle = rclpycycle.to_sec()
		lastscan = current_time	
		
		rclpycount = (len(raw_data)-headercodesize)/2

		scannum += 1	


		""" error monitoring, debug info """
		
		# output info every 10 scans
		if DEBUGOUTPUT and scannum % 10 == 0:
			msg = "SCAN "+str(scannum)+": cycle: "+str(cycle)+", count: "+str(count)
			node.get_logger().info(msg)
		
		# warn if firmware sample count doesn't match 
		if not rclpycount == count and scannum > 1 and DEBUGOUTPUT:
			msg = "SCAN "+str(scannum)+": count/data mismatch: "+ str( rclpycount-count )
			node.get_logger().warn(msg)

		# warn if sample count change
		if abs(count - lastcount) > 2 and scannum > 4 and DEBUGOUTPUT:
			msg = "SCAN "+str(scannum)+": count change from: "+ str(lastcount)+", to: "+str(count)
			node.get_logger().warn(msg)
		lastcount = count

		# reset device if low sample count
		if count<(nominalcycle/readInterval/2) and scannum > 4: 
			if DEBUGOUTPUT:  
				msg = "SCAN "+str(scannum)+": low data count: "+str(count)+", cycle: "+str(cycle)
				msg += ", scan dropped"
				node.get_logger().error(msg)

			if consecutivescandropped > 3:
				node.get_logger().error("resetting device")
				cleanup()
				break  

			del raw_data[:]
			ser.write(b"n\n0\n") # stop broadcast, lidar disable
			rclpy.sleep(0.1)
			ser.write(b"1\nb\n") # lidar enble, start broadcast
			consecutivescandropped += 1
			continue

		else:
			consecutivescandropped = 0


		
		if scannum <= 4: # drop 1st few scans while lidar spins up
			del raw_data[:]
			continue
			
			
		rebroadcastscan = None
		
		scan = LaserScan()
		
		scan.header.stamp = (current_time - rclpy.duration.Duration(seconds=cycle)).to_msg()
		lastscantime = current_time.nanoseconds/10**9 - cycle # seconds, floating point (used by rebroadcastscan only)
		
		scan.header.frame_id = frame_id

		scan.angle_min = 0.0
		rpmcycle = cycle
		scan.angle_max = (readInterval*(count-1))/rpmcycle * 2 * math.pi
		scan.angle_increment = (scan.angle_max-scan.angle_min) / (count-1)  	

		scan.time_increment =  readInterval
		scan.scan_time = readInterval * count # cycle # rclpycycle.to_sec()

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
		
		rclpy.spin_once(node, timeout_sec=0) # required for params to be set
		

def rebroadcast():
	global lastscantime, rebroadcastscan
	
	if lastscantime <= (node.get_clock().now().nanoseconds/10**9 - nominalcycle*2):
		lastscantime += nominalcycle
		rebroadcastscan.header.stamp = rclpy.time.Time(nanoseconds=(lastscantime*10**9)).to_msg()
		scan_pub.publish(rebroadcastscan)


def main(args=None):
	global scan_pub, node, ser, lidarrunning, rebroadcastscan

	# signal.signal(signal.SIGINT, cleanup) # this does nothing, probably overridden by ROS

	rclpy.init(args=args)
	node = rclpy.create_node('lidarbroadcast')
	
	declareParams()
	
	scan_pub = node.create_publisher(LaserScan, scan_topic, 3)
	node.create_subscription(ParameterEvent, 'parameter_events', parameterscallback, 10)

	# rospy.Subscriber("odom", Odometry, odomCallback) # TODO: make optional?

	while True:
		if ser == None: # and rclpy.ok(): #connect
			ser = device.usbdiscover("<id::xaxxonopenlidar>", 5)
			
			ser.write(b"y\n") # get version
			time.sleep(0.1)
			while ser.inWaiting() > 0:
				line = ser.readline().decode("utf-8").strip()
				line = "firmware: "+line
				node.get_logger().info(line)

			# set direction (1=CW RHR+ motor@bottom default, ROS default)
			ser.write(bytearray([ord("d"), DIR, ord("\n")]))

		if lidar_enable:
			updateRotateForwardOffset() # needs to be 1st
			updateForwardOffset()
			updateRPM()
			updateParkOffset()
			updateReadInterval()
			readlidar(ser) # blocking
			
		else:
			if lidarrunning:
				ser.write(b"f\n") # stop lidar
				lidarrunning = False
				node.get_logger().info("SCAN: lidar disabled");
				rebroadcastscan = scan	
			elif rebroadcastscan and BROADCASTLAST:
				rebroadcast()
			
		if not rclpy.ok():
			break
		else:
			device.removelockfile()
			
		rclpy.spin_once(node, timeout_sec=0.01)


if __name__ == '__main__':
	main()
