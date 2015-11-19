#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import PointCloud2


class TakePictureState(EventState):
	'''
	Stores the picture  of the given topic.

	#> Image     Image        The received pointcloud.

	<= done             The picture has been received and stored.

	'''

	def __init__(self):
		super(TakePictureState, self).__init__(outcomes = ['done'],    output_keys = ['Image'])
		self._topic = '/head_xtion/rgb/image_rect_color'
		self._sub = ProxySubscriberCached({self._topic:Image})
	
	
	def execute(self, userdata):
		if self._sub.has_msg(self._topic):
			userdata.Image = self._sub.get_last_msg(self._topic)
		print 'take_picture_state: took a picture'
		return 'done'
