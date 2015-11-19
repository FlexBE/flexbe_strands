#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import PointCloud2


class GetPointcloudState(EventState):
	'''
	Retrieves the latest pointcloud on the given topic.

	-- topic 		string 			The topic on which to listen for the pointcloud.
	-- timeout 		float 			Optional timeout in seconds of waiting for a pointclod.
									Set to 0 in order to wait forever.

	#> pointcloud 	PointCloud2		The received pointcloud.

	<= done 		Pointcloud has been received and is now available in userdata.
	<= timeout 		No pointcloud has been received, but maximum waiting time has passed.

	'''

	def __init__(self, topic, timeout = 0):
		super(GetPointcloudState, self).__init__(outcomes = ['done', 'timeout'],
													output_keys = ['pointcloud'])

		self._sub = ProxySubscriberCached({topic: PointCloud2})

		self._pcl_topic = topic

		self._timeout = timeout
		self._timeout_time = None


	def execute(self, userdata):
		if self._sub.has_msg(self._pcl_topic):
			userdata.pointcloud = self._sub.get_last_msg(self._pcl_topic)
			return 'done'

		if self._timeout_time < rospy.Time.now():
			return 'timeout'


	def on_enter(self, userdata):
		self._timeout_time = rospy.Time.now() + rospy.Duration(self._timeout)
		

	
		
