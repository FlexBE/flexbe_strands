#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from flir_pantilt_d46.msg import PtuGotoAction, PtuGotoGoal


class MoveCameraState(EventState):
	'''
	Moves the camera.

	># pan 		float 	Pan angle [-180, 180)
	># tilt 	float 	Tilt angle [-41, 41]

	<= done			Moved the camera.
	<= failed		Cannot send the action goal.

	'''

	def __init__(self):
		# See example_state.py for basic explanations.
		super(MoveCameraState, self).__init__(outcomes = ['done', 'failed'],
												 input_keys = ['pan', 'tilt'])

		self._topic = 'SetPTUState'
		self._client = ProxyActionClient({self._topic: PtuGotoAction})

		self._error = False


	def execute(self, userdata):
		if self._error:
			return 'failed'

		if self._client.has_result(self._topic):
			return 'done'


	def on_enter(self, userdata):
		goal = PtuGotoGoal()
		goal.pan = userdata.pan
        goal.tilt = userdata.tilt
        goal.pan_vel = 60
        goal.tilt_vel = 60

		self._error = False
		try:
			self._client.send_goal(self._topic, goal)
		except Exception as e:
			Logger.logwarn('Failed to send the camera movement command:\n%s' % str(e))
			self._error = True


	def on_exit(self, userdata):
		# Make sure that the action is not running when leaving this state.
		# A situation where the action would still be active is for example when the operator manually triggers an outcome.

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active action goal.')
		
