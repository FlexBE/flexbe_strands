#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from mary_tts.msg import maryttsAction, maryttsGoal


class SpeechOutputState(EventState):
	'''
	Lets the robot speak the given input string.

	># text 	string 	Text to output.

	<= done 			Speaking has finished.
	<= failed 			Failed to execute speaking.

	'''

	def __init__(self):
		super(SpeechOutputState, self).__init__(outcomes = ['done', 'failed'],
												input_keys = ['text'])

		self._topic = '/speak'
		self._client = ProxyActionClient({self._topic: maryttsAction})

		self._error = False


	def execute(self, userdata):
		if self._error:
			return 'failed'

		if self._client.has_result(self._topic):
			return 'done'

		
	def on_enter(self, userdata):
		goal = maryttsGoal()
		goal.text = userdata.text
		print 'speech_output_state: say : ', goal.text

		self._error = False
		try:
			self._client.send_goal(self._topic, goal)
		except Exception as e:
			Logger.logwarn('Failed to send the Sweep command:\n%s' % str(e))
			self._error = True


	def on_exit(self, userdata):
		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active action goal.')
		
