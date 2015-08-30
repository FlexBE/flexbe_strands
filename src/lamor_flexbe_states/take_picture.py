#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import PointCloud2


class TakePicture(EventState):
	'''
	Stores the picture  of the given topic.

	-- topic 		string 			The topic on which to listen for the Image.

	#> Image 	Image		The received pointcloud.

	<= done 			The picture has been received and stored.

	'''

	def __init__(self, topic):
		super(StorePointcloudState, self).__init__(outcomes = ['done'],	output_keys = ['Image'])
                self._topic = '/camera/image_raw' 
		self._sub = ProxySubscriberCached({self._topic:Image})


	def execute(self, userdata):
		
		if self._sub.has_msg(self._topic):
			userdata.image = self._sub.get_last_msg(self._topic)
			return 'done'
		

	
		
