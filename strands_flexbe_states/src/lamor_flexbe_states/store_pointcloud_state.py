#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import PointCloud2


class StorePointcloudState(EventState):
	'''
	Stores the latest pointcloud of the given topic.

	-- topic 		string 			The topic on which to listen for the pointcloud.

	#> pointcloud 	PointCloud2		The received pointcloud.

	<= done 			Pointcloud has been received and stored.

	'''

	def __init__(self, topic):
		super(StorePointcloudState, self).__init__(outcomes = ['done'],
													output_keys = ['pointcloud'])

		self._sub = ProxySubscriberCached({topic: PointCloud2})

		self._pcl_topic = topic


	def execute(self, userdata):
		
		if self._sub.has_msg(self._pcl_topic):
			userdata.pointcloud = self._sub.get_last_msg(self._pcl_topic)
			return 'done'
		

	
		
