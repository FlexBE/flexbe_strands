# -*- coding: utf-8 -*-
"""
Created on Mon Aug 31 17:49:55 2015

@author: tayyab
"""

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




class DetectFaceState(EventState):
    '''
    Displays the picture to the local home folder.

    ># Image    Image       The received Image
    <= done                 Displaying the Picture
    '''

    def __init__(self):
        super(DetectFaceState, self).__init__(outcomes = ['face_is_detected','face_is_not_detected'],    input_keys = ['Image'])
                

    def execute(self, userdata):
        return 'done'

    def on_enter(self,userdata):
        bridge =  CvBridge()
        cv_image = bridge.imgmsg_to_cv2(userdata.Image, desired_encoding="passthrough")
        faceCascade = cv2.CascadeClassifier(os.path.expanduser('~/haarcascade_frontalface_default.xml'))
        faces = faceCascade.detectMultiScale(cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY),scaleFactor=1.1,minNeighbors=5,minSize=(30, 30),flags=cv2.cv.CV_HAAR_SCALE_IMAGE)
        if (len(faces)>=1):
            return 'face_is_detected'
        else:
            return 'face_is_not_detected'