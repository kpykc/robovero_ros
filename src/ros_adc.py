#!/usr/bin/env python

"""Acquire and display ADC readings using the Arduino API.
"""
import roslib
roslib.load_manifest('robovero_ros')

import rospy

import sys,struct,time,os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../lib/robovero'))

#sys.path.append('/home/kp/rosws/catkin/src/robovero_ros/lib/robovero')

from robovero_ros.msg import robovero_adc
from robovero.extras import roboveroConfig
from robovero.arduino import analogRead, pinMode, AD0_0, AD0_1, AD0_2, AD0_3, AD0_5, AD0_6, AD0_7
from time import sleep

__author__ =			"Neil MacMunn"
__email__ =				"neil@gumstix.com"
__copyright__ = 	"Copyright 2010, Gumstix Inc"
__license__ = 		"BSD 2-Clause"
__version__ =			"0.1"


roboveroConfig()

class ADC:
	"""
	adc
	"""
	def __init__(self):
		print "robovero adc init"
		self.data = robovero_adc()
		self.pub = rospy.Publisher("/robovero/adc", robovero_adc)
		
	def spin(self):
		while not rospy.is_shutdown():
			self.time = rospy.Time.now()
						
			self.data.header.frame_id = "/imu_frame"
			self.data.header.seq = 1
			self.data.header.stamp = self.time
			
			self.data.AD0_0 = analogRead(AD0_0)
			self.data.AD0_1 = analogRead(AD0_1)
			self.data.AD0_2 = analogRead(AD0_2)
			self.data.AD0_3 = analogRead(AD0_3)
			self.data.AD0_4 = analogRead(AD0_5)
			self.data.AD0_5 = analogRead(AD0_6)
			self.data.AD0_6 = analogRead(AD0_7)
			
			self.pub.publish(self.data)
			
if __name__ == '__main__':
    rospy.init_node('ADC')
    robovero_adc = ADC()
	
    try:
        robovero_adc.spin()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass

  
