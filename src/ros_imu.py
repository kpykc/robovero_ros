#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Robot Control and Pattern Recognition Group, Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('robovero_ros')

import rospy

import sys,struct,time,os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../lib/robovero'))

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse

from PyKDL import Rotation

import serial
import math

from robovero.extras import Array, roboveroConfig
from robovero.lpc17xx_i2c import I2C_M_SETUP_Type, I2C_MasterTransferData, \
                            I2C_TRANSFER_OPT_Type
from robovero.lpc17xx_gpio import GPIO_ReadValue
from robovero.LPC17xx import LPC_I2C0
from robovero.lpc_types import Status
#import time

__author__ =      ["Neil MacMunn", "Danny Chan"]
__email__ =       "neil@gumstix.com"
__copyright__ =   "Copyright 2012, Gumstix Inc"
__license__ =     "BSD 2-Clause"
__version__ =     "0.1"

accel_ctrl_reg1 = 0x20
accel_ctrl_reg4 = 0x23
accel_x_low = 0x28
'''
accel_x_high = 0x29
accel_y_low = 0x2A
accel_y_high = 0x2B
accel_z_low = 0x2C
accel_z_high = 0x2D
'''

compass_cra_reg = 0x00
compass_crb_reg = 0x01
compass_mr_reg = 0x02
compass_x_high = 0x03
'''
compass_x_low = 0x04
compass_y_high = 0x05
compass_y_low = 0x06
compass_z_high = 0x07
compass_z_low = 0x08
'''

gyro_ctrl_reg1 = 0x20
gyro_ctrl_reg2 = 0x21
gyro_ctrl_reg3 = 0x22
gyro_ctrl_reg4 = 0x23
gyro_ctrl_reg5 = 0x24
gyro_status_reg = 0x27
gyro_x_low = 0x28
'''
gyro_x_high = 0x29
gyro_y_low = 0x2A
gyro_y_high = 0x2B
gyro_z_low = 0x2C
gyro_z_high = 0x2D
'''
gyro_fifo_ctrl_reg = 0x2E
print "Global vars"

class I2CDevice(object):
    def __init__(self, address):
        self.config = I2C_M_SETUP_Type()
        self.tx_data = Array(2, 1)
        self.rx_data = Array(1, 1)
        self.rx_data6 = Array(6, 1)
        self.config.sl_addr7bit = address
        self.config.tx_data = self.tx_data.ptr
        self.config.retransmissions_max = 3

    def readReg(self, register):
        self.tx_data[0] = register
        self.config.tx_length = 1
        self.config.rx_data = self.rx_data.ptr
        self.config.rx_length = 1 
        ret = I2C_MasterTransferData(LPC_I2C0, self.config.ptr,
                                     I2C_TRANSFER_OPT_Type.I2C_TRANSFER_POLLING)
        if ret == Status.ERROR:
            exit("I2C Read Error")    
        return self.rx_data[0]

    def read6Reg(self, register):
        self.tx_data[0] = register | 0b10000000 #MSB must be equal to 1 to read multiple bytes
        self.config.tx_length = 1
        self.config.rx_data = self.rx_data6.ptr
        self.config.rx_length = 6
        ret = I2C_MasterTransferData(LPC_I2C0, self.config.ptr,
                                     I2C_TRANSFER_OPT_Type.I2C_TRANSFER_POLLING)
        if ret == Status.ERROR:
            exit("I2C Read Error")    
        return self.rx_data6

    def writeReg(self, register, value):
        self.tx_data[0] = register
        self.tx_data[1] = value
        self.config.tx_length = 2
        self.config.rx_data = 0
        self.config.rx_length = 0
        ret = I2C_MasterTransferData(LPC_I2C0, self.config.ptr,
                                     I2C_TRANSFER_OPT_Type.I2C_TRANSFER_POLLING)
        if ret == Status.ERROR:
            exit("I2C Write Error")
        if self.readReg(register) != value:
            exit("I2C Verification Error")
        return None





class rIMU:
    """ 
    Class for interfacing with gyroscope on Elektron mobile robot.
    Responsible for retrieving data from device, calibration and
    calculating current orientation. 
    """

    def __init__(self):

        # open serial port
        #self.device = rospy.get_param('~device', '/dev/ttyUSB0')
        #self.baud = rospy.get_param('~baud', 38400)
        #self.ser = serial.Serial(self.device, self.baud, timeout=1)

        # reset variables
        #self.orientation = 0
        #self.bias = 0
        # Initialize pin select registers
        roboveroConfig()
        print "roboveroConfig"
        # configure accelerometer
        self.accelerometer = I2CDevice(0x18)
        self.accelerometer.writeReg(accel_ctrl_reg1, 0x27)
        self.accelerometer.writeReg(accel_ctrl_reg4, 0x00) # 
        
        # configure compass
        self.compass = I2CDevice(0x1E)
        self.compass.writeReg(compass_cra_reg, 0x18) # 75 Hz
        self.compass.writeReg(compass_crb_reg, 0x20) # +/- 1.3 gauss
        self.compass.writeReg(compass_mr_reg, 0) # continuous measurement mode
        
        # configure the gyro
        # from the L3G4200D Application Note:
        # 1. Write CTRL_REG2
        # 2. Write CTRL_REG3
        # 3. Write CTRL_REG4
        # 4. Write CTRL_REG6
        # 5. Write Reference
        # 6. Write INT1_THS
        # 7. Write INT1_DUR
        # 8. Write INT1_CFG
        # 9. Write CTRL_REG
        # 10. Write CTRL_REG
        self.gyro = I2CDevice(0x68)
        self.gyro.writeReg(gyro_ctrl_reg3, 0x08) # enable DRDY
        self.gyro.writeReg(gyro_ctrl_reg4, 0x80) # enable block data read mode
        self.gyro.writeReg(gyro_ctrl_reg1, 0x0F) # normal mode, enable all axes, 250dps

        self.calibrating = False

        self.frame_id = 'base_footprint'
        self.prev_time = rospy.Time.now()

        # publisher with imu data
        self.pub = rospy.Publisher("/robovero/imu/data", Imu)
        
        # rotation scale
        self.scale = rospy.get_param('~rot_scale', 1.0)
        
        # service for calibrating gyro bias
        rospy.Service("/robovero/imu/calibrate", Empty, self.calibrateCallback)
        
        # publisher with calibration state
        self.is_calibratedPublisher = rospy.Publisher('/robovero/imu/is_calibrated', Bool, latch=True)
        
        # We'll always just reuse this msg object:        
        self.is_CalibratedResponseMsg = Bool();

        # Initialize the latched is_calibrated state. 
        # At the beginning calibration is assumed to be done

        self.is_CalibratedResponseMsg.data = True;
        self.is_calibratedPublisher.publish(self.is_CalibratedResponseMsg)

    def calibrate(self):
        # calibration routine
        rospy.loginfo("Calibrating Gyro. Don't move the robot now")
        start_time = rospy.Time.now()
        cal_duration = rospy.Duration(5.0)
        offset = 0
        cnt = 0
        # flush input buffer to calibrate on newest data  
        #self.ser.flushInput()
        #while rospy.Time.now() < start_time + cal_duration:

            # get line from device
            #str = self.ser.readline()
            #nums = str.split()

            # check, if it was correct line
            #if (len(nums) != 5):
            #    continue

            #cnt += 1
            #gyro = int(nums[2])
            #ref = int(nums[3])
            #temp = int(nums[4])

            #val = ref-gyro;

            #offset += val;

        #self.bias = 1.0 * offset / cnt
        #rospy.loginfo("Gyro calibrated with offset %f"%self.bias)
        
        # Update the latched is_calibrated state:
        self.is_CalibratedResponseMsg.data = True
        self.is_calibratedPublisher.publish(self.is_CalibratedResponseMsg)
        
        return True



    def calibrateCallback(self, req):
        """The imu/calibrate service handler."""
          
        rospy.loginfo("Calibration request")
                
        self.calibrating = True
        
        return EmptyResponse()

    def twosComplement(self, low_byte, high_byte):
        """Unpack 16-bit twos complement representation of the result.
        """
        return (((low_byte + (high_byte << 8)) + 2**15) % 2**16 - 2**15)      

    def spin(self):
        self.prev_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.calibrating:
                self.calibrate()
                self.calibrating = False
                self.prev_time = rospy.Time.now()

            acceldata = self.accelerometer.read6Reg(accel_x_low)
            compassdata = self.compass.read6Reg(compass_x_high)
            gyrodata = self.gyro.read6Reg(gyro_x_low)

            # prepare Imu frame
            imu = Imu()
            imu.header.frame_id = self.frame_id
            self.linear_acceleration = Vector3();

            # get line from device
            #str = self.ser.readline()

            # timestamp
            imu.header.stamp = rospy.Time.now()

            #nums = str.split()

            # check, if it was correct line
            #if (len(nums) != 5):
            #    continue

            self.linear_acceleration.x = self.twosComplement(acceldata[0], acceldata[1]) #/16384.0
            self.linear_acceleration.y = self.twosComplement(acceldata[2], acceldata[3]) #/16384.0
            self.linear_acceleration.z = self.twosComplement(acceldata[4], acceldata[5]) #/16384.0

            imu.orientation.x = self.twosComplement(compassdata[1], compassdata[0]) #/1055.0,
            imu.orientation.y = self.twosComplement(compassdata[3], compassdata[2]) #/1055.0,
            imu.orientation.z = self.twosComplement(compassdata[5], compassdata[4]) #/950.0

            imu.angular_velocity.x = self.twosComplement(gyrodata[0], gyrodata[1])
            imu.angular_velocity.y = self.twosComplement(gyrodata[2], gyrodata[3])
            imu.angular_velocity.z = self.twosComplement(gyrodata[4], gyrodata[5])

            #gyro = int(nums[2])
            #ref = int(nums[3])
            #temp = int(nums[4])

            #val = (ref-gyro - self.bias) * 1000 / 3 / 1024 * self.scale

            #imu.angular_velocity.x = 0
            #imu.angular_velocity.y = 0
            #imu.angular_velocity.z = val * math.pi / 180
            imu.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]
            imu.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.1]

            self.orientation += imu.angular_velocity.z * (imu.header.stamp - self.prev_time).to_sec()
            self.prev_time = imu.header.stamp
            (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w) = Rotation.RotZ(self.orientation).GetQuaternion()
            self.pub.publish(imu)



if __name__ == '__main__':
    rospy.init_node('IMU')
    rimu = rIMU()
    rimu.calibrate()

    try:
        rimu.spin()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass

