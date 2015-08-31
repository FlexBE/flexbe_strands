#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyPublisher

from std_msgs.msg import String


class ShowPictureWebinterfaceState(EventState):
    '''
    Displays the picture in a web interface

    ># Image    Image       The received Image
    <= done                 Displaying the Picture
    '''

    def __init__(self):
        super(ShowPictureWebinterfaceState, self).__init__(outcomes = ['tweet', 'forget'],
                                                            input_keys = ['image_name'])

        self._pub_topic = '/webinterface/display_picture'
        self._pub = ProxyPublisher({self._pub_topic: String})

        self._sub_topic = '/webinterface/dialog_feedback'
        self._sub = ProxySubscriberCached({self._sub_topic: String})
                

    def execute(self, userdata):
        if self._sub.has_msg(self._sub_topic):
            msg = self._sub.get_last_msg(self._sub_topic)

            if msg.data == 'yes':
                return 'tweet'
            else:
                return 'forget'

    def on_enter(self,userdata):
        self._sub.remove_last_msg(self._sub_topic)
        self._pub.publish(self._pub_topic, String(userdata.image_name))

