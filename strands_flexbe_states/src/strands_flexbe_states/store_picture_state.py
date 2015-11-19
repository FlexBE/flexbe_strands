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
	Stores the provided image to a local folder.

	-- folder 	string 	Path of the folder where to store the image, defaults to the home folder.

	># image 	Image 	The received Image.

	#> filepath string 	Path of the image file which has been created.

	<= done 			The picture has been received and stored.

	'''

	def __init__(self, folder = '~'):
		super(StorePictureState, self).__init__(outcomes = ['done'],
												input_keys = ['image'],
												output_keys=['filepath'])
			
		self._filepath = None
		self._folder = folder

	def execute(self, userdata):
		userdata.filepath = self._filepath
		return 'done'

	def on_enter(self,userdata):
		bridge =  CvBridge()
		cv_image = bridge.imgmsg_to_cv2(userdata.image, desired_encoding="passthrough")
		self._filepath = os.path.expanduser(os.path.join(self._folder, 'picture_'+str(rospy.Time.now())+'.jpg'))
		cv2.imwrite(self._filepath,cv_image)
		Logger.loginfo('Created image file:\n%s' % self._filepath)

