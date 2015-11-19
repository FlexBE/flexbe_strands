#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from cloud_merge.msg import SweepAction, SweepGoal


class MetricSweepState(EventState):
	'''
	Robot performs a metric sweep at its current location.

	-- sweep_type	string 	Type of the sweep to do (select one of the provided options).

	<= sweeped 		Sweep has been completed.
	<= failed		There has been an error when sweeping.

	'''

	COMPLETE = 'complete'
	MEDIUM = 'medium'
	SHORT = 'short'
	SHORTEST = 'shortest'

	def __init__(self, sweep_type):
		super(MetricSweepState, self).__init__(outcomes = ['sweeped', 'failed'])

		self._sweep_type = sweep_type

		self._topic = '/do_sweep'
		self._client = ProxyActionClient({self._topic: SweepAction})

		self._error = False


	def execute(self, userdata):
		if self._error:
			return 'failed'

		if self._client.has_result(self._topic):
			result = self._client.get_result(self._topic)

			if result.success:
				return 'sweeped'
			else:
				return 'failed'

		
	def on_enter(self, userdata):
		goal = SweepGoal()
		goal.type = self._sweep_type

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
		
