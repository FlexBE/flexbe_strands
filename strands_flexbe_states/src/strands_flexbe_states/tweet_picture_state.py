#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from strands_tweets.msg import SendTweetAction, SendTweetGoal
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class TweetPictureState(EventState):
    '''
    State for tweeting a picture and text

    ># picture_path     string      Path to the picture to be tweeted.
    ># tweet_text       string      Text to be tweeted, has to be at most 140 characters long.

    <= picture_tweeted              Successfully tweeted picture and text.
                                    If text has been too long, a placeholder was used.
    <= tweeting_failed              The twitter node indicated that the message could not be twittered.
    <= command_error                Failed to send the tweet command to the twitter node.

    '''

    def __init__(self):
        super(TweetPictureState, self).__init__(outcomes = ['picture_tweeted', 'tweeting_failed','command_error'],
                                                input_keys = ['picture_path', 'tweet_text'])

        self._topic = 'strands_tweets'
        self._client = ProxyActionClient({self._topic: SendTweetAction})

        self._error = False

    def execute(self, userdata):
        if self._error:
            return 'command_error'

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            if result.success:
                return 'picture_tweeted'

            else:
                return 'tweeting_failed'
      

    def on_enter(self, userdata):
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
            Logger.logwarn('Failed to send the TweetPictureState command:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
        
