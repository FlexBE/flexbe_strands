#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import Image


class TakePictureState(EventState):
	'''
	Takes a picture.

	#> image     Image        The taken picture.

	<= done             The picture has been received and stored.

	'''

	def __init__(self):
		super(TakePictureState, self).__init__(outcomes = ['done'],
												output_keys = ['image'])
		
		self._topic = '/head_xtion/rgb/image_rect_color'
		self._sub = ProxySubscriberCached({self._topic: Image})
	
	
	def execute(self, userdata):
		if self._sub.has_msg(self._topic):
			userdata.image = self._sub.get_last_msg(self._topic)
			return 'done'
