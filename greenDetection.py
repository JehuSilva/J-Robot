#!/usr/bin/env python 
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 
from std_msgs.msg import Int32

pubx = rospy.Publisher('GreenX', Int32, queue_size=10)
puby = rospy.Publisher('GreenY',Int32,queue_size=10)

def nothing(x):
    pass
def color_detect(img):
    
    lower = np.array([75,97,61])
    upper = np.array([88,245,199])
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    kernel = np.ones((10,10), np.uint8)
    mask = cv.inRange(hsv,lower,upper)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    num,labels= cv.connectedComponents(mask)
    #for i in range(1,num):
    #    img2=np.array(3, dtype=np.uint8)
    #    img2[label=labels]

    #markers=markers+1
    #markers[mask==255]=0
    #markers = cv.watershed(img,markers)
    #img[markers == -1] = [255,0,0]


    moments = cv.moments(mask)
    area = moments['m00']
    edged = cv.Canny(mask,35,125)
    image, cnt, hierarchy = cv.findContours(edged, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

    if(area > 20000):
        x = int(moments['m10']/moments['m00'])
        y = int(moments['m01']/moments['m00'])
        pubx.publish(x)
        puby.publish(y)
        #print x,y
        cv.rectangle(img,(x,y),(x+2,y+2),(0,0,255),2)
        
    cv.imshow('Green Detection',edged)
    #cv.imshow('mask',mask)
    #cv.imshow('image',img)
    cv.waitKey(1)

def callback(msg):
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg,"bgr8")
        color_detect(orig)
    except CvBridgeError as e:
        print(e)

rospy.init_node('GreenColorDetection')
sub = rospy.Subscriber('/axis_videocap/image_raw',Image, callback)
#sub = rospy.Subscriber('/usb_cam/image_raw',Image, callback)
rospy.spin()