#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from recognition_srv_definitions.srv import recognizeRequest

class RecognizeObjectState(EventState):
	'''
	Recognizes an object in the given pointcloud.

	-- object_name	string 		Name of the object to look for.

	># pointcloud 	PointCloud2	The perception data to work on.

	<= detected 				Object is detected.
	<= not_detected 			Object is not detected.
	<= failed 					Something regarding the service call went wrong.

	'''

	def __init__(self, object_name):
		super(RecognizeObjectState, self).__init__(outcomes = ['detected', 'not_detected', 'failed'],
													input_keys = ['pointcloud'])

		self._srv_topic = '/recognition_service/sv_recognition'
		self._srv = ProxyServiceCaller({self._srv_topic: recognizeRequest})

		self._object_name = object_name
		self._srv_result = None
		self._failed = False


	def execute(self, userdata):
		if self._failed or self._srv_result is None:
			return 'failed'

		if self._object_name in [s.data for s in self._srv_result.ids]:
			return 'detected'
		else:
			return 'not_detected'


	def on_enter(self, userdata):

		srv_request = recognizeRequest(cloud=userdata.pointcloud)

		self._failed = False
		try:
			self._srv_result = self._srv.call(self._srv_topic, srv_request)
			print self._srv_result
		except Exception as e:
			rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
			self._failed = True

	
		
