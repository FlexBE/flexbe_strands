#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from geometry_msgs.msg import PoseStamped


class DetectPersonState(EventState):
	'''
	Detects the nearest person and provides their pose.

	-- wait_timeout	float 			Time (seconds) to wait for a person before giving up.

	#> person_pose 	PoseStamped 	Pose of the nearest person if one is detected, else None.

	<= detected 		Detected a person.
	<= not_detected		No person detected, but time ran out.

	'''


	def __init__(self, wait_timeout):
		super(DetectPersonState, self).__init__(outcomes = ['detected', 'not_detected'],
												output_keys = ['person_pose'])

		self._wait_timeout = rospy.Duration(wait_timeout)

		self._topic = '/people_tracker/pose'
		self._sub = ProxySubscriberCached({self._topic: PoseStamped})

		self._start_waiting_time = None


	def execute(self, userdata):
		if rospy.Time.now() > self._start_waiting_time + self._wait_timeout:
			userdata.person_pose = None
			return 'not_detected'

		if self._sub.has_msgs(self._topic):
			userdata.person_pose = self._sub.get_last_msg(self._topic)
			return 'detected'

		
	def on_enter(self, userdata):
		self._start_waiting_time = rospy.Time.now()
		
