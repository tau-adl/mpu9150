#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
import pigpio
import time
import binascii
import numpy as np
from timeit import default_timer as timer


# Global Variables
MPU_9150_I2C_ADDRESS_1     =     0x69    # Base address of the Drotek board
MPU_9150_I2C_ADDRESS_2     =     0x68    # Base address of the SparkFun board
MPU_9150_SMPRT_DIV         =     0x19    # Gyro sampling rate divider
MPU_9150_DEFINE            =     0x1A    # Gyro and accel configuration
MPU_9150_GYRO_CONFIG       =     0x1B    # Gyroscope configuration
MPU_9150_ACCEL_CONFIG      =     0x1C    # Accelerometer configuration
MPU_9150_FIFO_EN           =     0x23    # FIFO buffer control
MPU_9150_INT_PIN_CFG       =     0x37    # Bypass enable configuration
MPU_9150_INT_ENABLE        =     0x38    # Interrupt control
MPU_9150_ACCEL_XOUT_H      =     0x3B    # Accel X axis High
MPU_9150_ACCEL_XOUT_L      =     0x3C    # Accel X axis Low
MPU_9150_ACCEL_YOUT_H      =     0x3D    # Accel Y axis High
MPU_9150_ACCEL_YOUT_L      =     0x3E    # Accel Y axis Low
MPU_9150_ACCEL_ZOUT_H      =     0x3F    # Accel Z axis High
MPU_9150_ACCEL_ZOUT_L      =     0x40    # Accel Z axis Low
MPU_9150_GYRO_XOUT_H       =     0x43    # Gyro X axis High
MPU_9150_GYRO_XOUT_L       =     0x44    # Gyro X axis Low
MPU_9150_GYRO_YOUT_H       =     0x45    # Gyro Y axis High
MPU_9150_GYRO_YOUT_L       =     0x46    # Gyro Y axis Low
MPU_9150_GYRO_ZOUT_H       =     0x47    # Gyro Z axis High
MPU_9150_GYRO_ZOUT_L       =     0x48    # Gyro Z axis Low
MPU_9150_USER_CTRL         =     0x6A    # User control
MPU_9150_PWR_MGMT_1        =     0x6B    # Power management 1

MPU_9150_I2C_MAGN_ADDRESS  =     0x0C    # Address of the magnetometer in bypass mode
MPU_9150_WIA               =     0x00    # Mag Who I Am
MPU_9150_AKM_ID            =     0x48    # Mag device ID
MPU_9150_ST1               =     0x02    # Magnetometer status 1
MPU_9150_HXL               =     0x03    # Mag X axis Low
MPU_9150_HXH               =     0x04    # Mag X axis High
MPU_9150_HYL               =     0x05    # Mag Y axis Low
MPU_9150_HYH               =     0x06    # Mag Y axis High
MPU_9150_HZL               =     0x07    # Mag Z axis Low
MPU_9150_HZH               =     0x08    # Mag Z axis High
MPU_9150_ST2               =     0x09    # Magnetometer status 2
MPU_9150_CNTL              =     0x0A    # Magnetometer control


IMUPI_BLOCK_SIZE           =     6	 # Accelerometer, Gyro block size
IMUPI_MES_SIZE             =     14	 # Total block size for single read
IMUPI_NB_AXIS              =     3
IMUPI_I2C_AUTO_INCREMENT   =     0x80
IMUPI_A_GAIN               =     6.103515625e-05
IMUPI_G_GAIN               =     0.030487804878049
IMUPI_M_GAIN               =     0.3001221001221001
IMUPI_GOFF_NB_ITER         =     500

IMUPI_NO_ERROR             =     0
IMUPI_I2C_OPEN_ERROR       =     1
IMUPI_I2C_DEV_NOT_FOUND    =     2
IMUPI_I2C_WRITE_ERROR      =     3
IMUPI_I2C_READ_ERROR       =     4
IMUPI_INIT_ERROR           =     5

M_PI                       =     3.14159265358979323846
M_G                        =     9.81

GYRO_OFF_X = 0
GYRO_OFF_Y = 0
GYRO_OFF_Z = 0

mpu_seq = 0
trig_seq = 0
init_time = False
read_time = 0
td = - rospy.Duration.from_sec(0.002)  # Accel latency is 2ms, gyro is 1.9ms

def init_mpu9150(pigpio_ptr):

	h = pigpio_ptr.i2c_open(1, MPU_9150_I2C_ADDRESS_2)
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_SMPRT_DIV, 0x04)  #200Hz sample rate
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_DEFINE, 0x01)  #No ext sync, no low pass filter
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_ACCEL_CONFIG, 0x00)  #Accel range
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_GYRO_CONFIG, 0x10)  #Gyro range
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_FIFO_EN, 0x00)  #Disable FIFO
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_INT_PIN_CFG, 0x02)  #Bypass mode enabled
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_INT_ENABLE, 0x01)  #Enable/Disable interrupts
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_USER_CTRL, 0x00)  #No FIFO and no I2C slaves
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_PWR_MGMT_1, 0x00)  #No power management, internal clock source

	#sleep for a second to let device initialize
	time.sleep(1)

        return h

def read_accel_data(pigpio_ptr, handle):
        data = pigpio_ptr.i2c_read_i2c_block_data(handle, MPU_9150_ACCEL_XOUT_H, IMUPI_BLOCK_SIZE)[1]


def read_gyro_data(pigpio_ptr, handle):
        data = pigpio_ptr.i2c_read_i2c_block_data(handle, MPU_9150_GYRO_XOUT_H, IMUPI_BLOCK_SIZE)[1]

def read_data(pigpio_ptr, handle):
	global init_time, read_time, td
	#start=timer()
	if not init_time:
		read_time = rospy.Time.now() 
		init_time = True
	else:
		read_time = read_time + rospy.Duration.from_sec(0.005)
	data = pigpio_ptr.i2c_read_i2c_block_data(handle, MPU_9150_ACCEL_XOUT_H, IMUPI_MES_SIZE)[1]
	#end=timer()
	#elapsed = end-start
	#td = rospy.Duration.from_sec(elapsed/2)
	#td = rospy.Duration.from_sec(0.002)
	
        
	return data, read_time #+ td

def decode_regs_encode_msg(data, header=None, calibration=False):
	global td

	ax = np.int16(int(binascii.hexlify(data[0:2]), 16)) * IMUPI_A_GAIN * M_G
	ay = np.int16(int(binascii.hexlify(data[2:4]), 16)) * IMUPI_A_GAIN * M_G
	az = np.int16(int(binascii.hexlify(data[4:6]), 16)) * IMUPI_A_GAIN * M_G

	gx = np.int16(int(binascii.hexlify(data[8:10]), 16)) * IMUPI_G_GAIN * M_PI/180 - GYRO_OFF_X
	gy = np.int16(int(binascii.hexlify(data[10:12]), 16)) * IMUPI_G_GAIN * M_PI/180 - GYRO_OFF_Y
	gz = np.int16(int(binascii.hexlify(data[12:14]), 16)) * IMUPI_G_GAIN * M_PI/180 - GYRO_OFF_Z
	
	if calibration == False:
		msg = Imu()
		msg.linear_acceleration.x = ax
		msg.linear_acceleration.y = ay
		msg.linear_acceleration.z = az

		msg.angular_velocity.x = gx
		msg.angular_velocity.y = gy
		msg.angular_velocity.z = gz

		msg.header = header
		msg.header.stamp = msg.header.stamp + td
		return msg

	else:
		#print('ax: {0},  ay: {1},  az: {2},	gx: {3},  gy: {4},  gz: {5}'.format(ax, ay, az, gx, gy, gz))
		return ax, ay, az, gx, gy, gz

def calc_gyro_offset():
	print("Calculating Gyro Offsets...")
	_off1 = 0
	_off2 = 0
	_off3 = 0
	for i in range(IMUPI_GOFF_NB_ITER):
		data, _ = read_data(pi, h)
		res=decode_regs_encode_msg(data, calibration=True)
		_off1 += res[3]
		_off2 += res[4]
		_off3 += res[5]
	
	GYRO_OFF_X = _off1/IMUPI_GOFF_NB_ITER
	GYRO_OFF_Y = _off2/IMUPI_GOFF_NB_ITER
	GYRO_OFF_Z = _off3/IMUPI_GOFF_NB_ITER
	print("X: {0},		Y: {1},		Z: {2}".format(GYRO_OFF_X, GYRO_OFF_Y, GYRO_OFF_Z))


def cbf(gpio, level, tick):
	global mpu_seq, trig_seq

	data, corrected_stamp = read_data(pi, h)
	mh = Header()
	mh.stamp = corrected_stamp
	mh.frame_id = 'mpu9150_frame'
	mh.seq = mpu_seq
	if (trig_flag and mpu_seq % trig_div == 0):
		pi.gpio_trigger(gpio_trig_num, 100, 1)
		tmp = Header()
		tmp = mh
		tmp.seq = trig_seq
		trig_seq += 1
		pub_trig.publish(mh)
	res=decode_regs_encode_msg(data, header=mh)
	pub.publish(res)
	mpu_seq += 1

pub = rospy.Publisher('/imu9150', Imu, queue_size=10)
pub_trig = rospy.Publisher('/trigger', Header, queue_size = 10)
rospy.init_node('mpu9150', anonymous=True)

full_param_name = rospy.search_param('host_sample_rate')
if full_param_name == None:
	full_param_name = 'host_sample_rate'
read_rate = rospy.get_param(full_param_name, 100)
rate = rospy.Rate(read_rate)

full_param_name = rospy.search_param('trigger_enable')
if full_param_name == None:
	full_param_name = 'trigger_enable'
trigger_enable = rospy.get_param(full_param_name, True)
trig_flag = trigger_enable

full_param_name = rospy.search_param('trigger_divider')
if full_param_name == None:
	full_param_name = 'trigger_divider'
trigger_divider = rospy.get_param(full_param_name, 8)
trig_div = trigger_divider

full_param_name = rospy.search_param('gpio_num')
if full_param_name == None:
	full_param_name = 'gpio_num'
gpio_num = rospy.get_param(full_param_name, 23)
gpio_trig_num = gpio_num

pi = pigpio.pi()

if trig_flag:
	pi.set_mode(gpio_trig_num, pigpio.OUTPUT)
	pi.write(gpio_trig_num, 0)
	pi.set_pull_up_down(gpio_trig_num, pigpio.PUD_DOWN)

h = init_mpu9150(pi)
#calc_gyro_offset()
cb1 = pi.callback(24, pigpio.RISING_EDGE, cbf)
while not rospy.is_shutdown():

	#data, corrected_stamp = read_data(pi, h)
	
	#mh = Header()
	#mh.stamp = corrected_stamp
	#mh.frame_id = 'mpu9150_frame'
	#mh.seq = mpu_seq
	#if (trig_flag and mpu_seq % trig_div == 0):
	#	pi.gpio_trigger(gpio_trig_num, 100, 1)
	#	tmp = Header()
	#	tmp = mh
	#	tmp.seq = trig_seq
	#	trig_seq += 1
	#	pub_trig.publish(mh)
	#res=decode_regs_encode_msg(data, header=mh)
	#pub.publish(res)
	#mpu_seq += 1
	#rate.sleep()
	pass





