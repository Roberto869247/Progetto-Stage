import rospy
from rospy import msg
from rospy import exceptions
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

import time
import serial
import adafruit_bno08x
from adafruit_bno08x.uart import BNO08X_UART


def publisher():
	msg = Imu()
	msg.header.frame_id = "Imu"
	msg.header.stamp = rospy.rostime.get_rostime()
	msg.header.seq = seq + 1 

	while not rospy.is_shutdown():
		accel_x, accel_y, accel_z = bno.acceleration
		gyro_x, gyro_y, gyro_z = bno.gyro
		dqx, dqy, dqz, dqw = bno.quaternion

		msg.linear_acceleration.x = accel_x
		msg.linear_acceleration.y = accel_y
		msg.linear_acceleration.z = accel_z

		msg.angular_velocity.x = gyro_x
		msg.angular_velocity.y = gyro_y
		msg.angular_velocity.z = gyro_z

		msg.orientation = Quaternion(dqx, dqy, dqz, dqw)
		
		pub.publish(msg)
		rate.sleep()

if __name__ == "__main__":
	pub = rospy.Publisher('/imu/data', Imu, queue_size = 1000)
	rospy.init_node('publisher', anonymous = True)    
	rate = rospy.Rate(50) # 50hz
	seq = 0

	while not rospy.is_shutdown(): 
		try:
			uart = serial.Serial("/dev/ttyUSB0", baudrate = 3000000)
		except Exception:
			rospy.logerr("Device not found, stopping, retrying...")
			time.sleep(3)
		else:
			break

	try:
		bno = BNO08X_UART(uart)
		bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
		bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
		#bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
		bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
	except Exception:	
		pass

	try:
		publisher()
	except rospy.ROSInterruptException:
		pass