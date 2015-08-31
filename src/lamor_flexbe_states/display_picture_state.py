# -*- coding: utf-8 -*-
"""
Created on Mon Aug 31 09:54:16 2015

@author: tayyab
"""

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
import os 
import sys
from sensor_msgs.msg import PointCloud2




class DisplayPictureState(EventState):
    '''
    Displays the picture to the local home folder.

    ># Image    Image       The received Image
    <= done                 Displaying the Picture
    '''

    def __init__(self):
        super(DisplayPictureState, self).__init__(outcomes = ['done'],    input_keys = ['Image'])
                

    def execute(self, userdata):
        return 'done'

    def on_enter(self,userdata):
        bridge =  CvBridge()
        cv_image = bridge.imgmsg_to_cv2(userdata.Image, desired_encoding="passthrough")
        cv2.namedWindow("ECMR_2015_Official_Photographer",cv2.WINDOW_NORMAL)
        cv2.imshow('ECMR_2015_Official_Photographer',cv_image)
        print 'Displaying the Picture'        
        #wait for keypress()
        cv2.waitKey(30000)
        cv2.destroyAllWindows()
        cv2.waitKey(1)

