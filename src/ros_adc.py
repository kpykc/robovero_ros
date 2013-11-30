#!/usr/bin/env python

"""Acquire and display ADC readings using the Arduino API.
"""
import roslib
roslib.load_manifest('robovero_ros')

import rospy

import sys,struct,time,os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../lib/robovero'))
#sys.path.append('/home/kp/rosws/catkin/src/robovero_ros/lib/robovero')

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

# Import custom message data and dynamic reconfigure variables.
from robovero_ros.msg import adc
from robovero_ros.cfg import ADCRateConfig as ConfigType


from robovero.extras import roboveroConfig
from robovero.arduino import analogRead, pinMode, AD0_0, AD0_1, AD0_2, AD0_3, AD0_5, AD0_6, AD0_7
from time import sleep

__author__ =			"Neil MacMunn"
__email__ =				"neil@gumstix.com"
__copyright__ = 	"Copyright 2010, Gumstix Inc"
__license__ = 		"BSD 2-Clause"
__version__ =			"0.1"


roboveroConfig()

class robovero_adc:
	"""
	adc
	"""
	def __init__(self):
		print "robovero adc init"
		
		rate = float(rospy.get_param('~rate', '10'))
		rospy.loginfo('rate = %d', rate)
		self.data = adc()
		self.server = DynamicReconfigureServer(ConfigType, self.reconfigure_cb)
		self.pub = rospy.Publisher("/robovero/adc", adc)
		
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
			if self.rate:
				rospy.sleep(1/self.rate)
			else:
				rospy.sleep(0.1)
           
	def reconfigure_cb(self, config, level):
		self.rate = config["rate"]
		return config
			
if __name__ == '__main__':
    rospy.init_node('robovero_adc0')
    robovero_adc0 = robovero_adc()
	
    try:
        robovero_adc0.spin()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass

  
