#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from stalking.msg import StalkPoseAction, StalkPoseGoal
from geometry_msgs.msg import PoseStamped


class ApproachPersonState(EventState):
	'''
	Actionlib actions are the most common basis for state implementations
	since they provide a non-blocking, high-level interface for robot capabilities.
	The example is based on the DoDishes-example of actionlib (see http://wiki.ros.org/actionlib).
	This time we have input and output keys in order to specify the goal and possibly further evaluate the result in a later state.

	-- dishes_to_do int 	Expected amount of dishes to be cleaned.

	># dishwasher 	int 	ID of the dishwasher to be used.

	#> cleaned 		int 	Amount of cleaned dishes.

	<= cleaned_some 		Only a few dishes have been cleaned.
	<= cleaned_enough		Cleaned a lot of dishes.
	<= command_error		Cannot send the action goal.

	'''

	def __init__(self):
		# See example_state.py for basic explanations.
		super(ApproachPersonState, self).__init__(outcomes = ['goal_reached', 'goal_failed','command_error'], input_keys = ['person_pose'])

		# Create the action client when building the behavior.
		# This will cause the behavior to wait for the client before starting execution
		# and will trigger a timeout error if it is not available.
		# Using the proxy client provides asynchronous access to the result and status
		# and makes sure only one client is used, no matter how often this state is used in a behavior.
		self._topic = 'stalk_pose'
		self._client = ProxyActionClient({self._topic: StalkPoseAction}) # pass required clients as dict (topic: type)
		self._timeoutTime = None

		self._personTopic = '/upper_body_detector/closest_bounding_box_centre'

		# It may happen that the action client fails to send the action goal.
		self._error = False

	def execute(self, userdata):
		# While this state is active, check if the action has been finished and evaluate the result.
		print 'approach_person_state: returning goal_reached'
		return 'goal_reached'
		# Check if the client failed to send the goal.
		if self._error:
			return 'command_error'

		if rospy.Time.now() > self._timeoutTime:
			return 'goal_failed'

		# Check if the action has been finished
		if self._client.has_result(self._topic):
			result = self._client.get_result(self._topic)

			if result.success:
				return 'goal_reached'

			else:
				return 'goal_failed'

		# If the action has not yet finished, no outcome will be returned and the state stays active.
		

	def on_enter(self, userdata):
		# When entering this state, we send the action goal once to let the robot start its work.

		# As documented above, we get the specification of which dishwasher to use as input key.
		# This enables a previous state to make this decision during runtime and provide the ID as its own output key.

		# Create the goal.
		goal = StalkPoseGoal() 

		goal.runtime_sec = 60
		goal.topic_name = self._personTopic
		goal.target = userdata.person_pose
		self._error = False

		self._timeoutTime = rospy.Time.now() + rospy.Duration(20)

		#try:
		#	self._client.send_goal(self._topic, goal)
		#except Exception as e:
				# Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
				# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
		#	Logger.logwarn('Failed to send the ApproachPerson command:\n%s' % str(e))
		#	self._error = True

	def on_exit(self, userdata):
		# Make sure that the action is not running when leaving this state.
		# A situation where the action would still be active is for example when the operator manually triggers an outcome.

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active action goal.')
		
