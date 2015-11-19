#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
import os 
import sys
from sensor_msgs.msg import PointCloud2


class StorePictureState(EventState):
	'''
	Stores the picture to the local home folder.

	># Image    Image       The received Image
	<= done             The picture has been received and stored.

	'''

	def __init__(self):
		super(StorePictureState, self).__init__(outcomes = ['done'],    input_keys = ['Image'], output_keys=['filename'])
			
		self._filename = None

	def execute(self, userdata):
		userdata.filename = self._filename
		return 'done'

	def on_enter(self,userdata):
		bridge =  CvBridge()
		cv_image = bridge.imgmsg_to_cv2(userdata.Image, desired_encoding="passthrough")
		self._filename = os.path.expanduser('~/picture_'+str(rospy.Time.now())+'.jpg')
		print 'Saving file to ' , self._filename
		cv2.imwrite(self._filename,cv_image)
		print 'Picture has been saved to the home folder'   

