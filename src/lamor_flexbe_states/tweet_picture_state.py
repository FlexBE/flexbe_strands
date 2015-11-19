#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from strands_tweets.msg import SendTweetAction, SendTweetGoal
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class TweetPictureState(EventState):
    '''
    State for tweeting a picture and text

    '''

    def __init__(self):
        # See example_state.py for basic explanations.
        super(TweetPictureState, self).__init__(outcomes = ['picture_tweeted', 'tweeting_failed','command_error'], input_keys = ['picture_path', 'tweet_text'])

        # Create the action client when building the behavior.
        # This will cause the behavior to wait for the client before starting execution
        # and will trigger a timeout error if it is not available.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        self._topic = 'strands_tweets'
        self._client = ProxyActionClient({self._topic: SendTweetAction}) # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._error = False

    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return 'command_error'

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            if result.success:
                return 'picture_tweeted'

            else:
                return 'tweeting_failed'

        # If the action has not yet finished, no outcome will be returned and the state stays active.
        

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # As documented above, we get the specification of which dishwasher to use as input key.
        # This enables a previous state to make this decision during runtime and provide the ID as its own output key.

        # Create the goal.
        tweet = SendTweetGoal() 
        tweet.text = userdata.tweet_text
        
        if len(userdata.tweet_text) > 140:
            tweet.text = '#LAMoR15 #ECMR15 I just told a too looong joke, stupid tweet length!'

        tweet.with_photo = True
        img = cv2.imread(userdata.picture_path)

        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        tweet.photo = image_message

        try:
            self._client.send_goal(self._topic, tweet)
        except Exception as e:
                # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
                # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn('Failed to send the TweetPictureState command:\n%s' % str(e))
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
        
