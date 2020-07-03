#!/usr/bin/env python
import rospy
import cv2 as cv 
import numpy as np 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist 
import cv_bridge
import matplotlib.pyplot as plt

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.twist = Twist()
    
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.read = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)



    def callback(self, data):
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])

        low_green = np.array([25, 52, 72])
        high_green = np.array([102, 255, 255])

        lower_red = np.array([0,120,70])
        upper_red = np.array([10,255,255])

        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])
                
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        height = img.shape[0]
        width = img.shape[1]
        top = 3*height/4-20
        bottom = top + 40
        
        hsv1 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        
        img[:top,:,:] = 0
        img[bottom:,:,:] = 0

        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        yellow_mask = cv.inRange(hsv , yellow_lower, yellow_upper)
        red_mask = cv.inRange(hsv, lower_red, upper_red)
        green_mask = cv.inRange(hsv, low_green, high_green)
        blue_mask = cv.inRange(hsv, lower_blue, upper_blue)

        #yellow_mask[:top,:] = 0
        #yellow_mask[bottom:,:] = 0

        #red_mask[:top,:] = 0
        #red_mask[bottom:,:] = 0

        mask = cv.bitwise_or(green_mask, yellow_mask)
        mask = cv.bitwise_or(mask, red_mask)

        mask[:,:width/5]=0
        mask[:,4*width/5:]=0

        M_b = cv.moments(blue_mask)
        M_y = cv.moments(mask)

        self.twist.linear.x = 0
        self.twist.angular.z =0
        self.twist.linear.y=0

        c_x=0
        c_y=0

        if (M_b['m00']!=0):
            c_x = int(M_b['m10']/M_b['m00'])
            c_y = int(M_b['m01']/M_b['m00'])
        elif (M_y['m00']!=0):
            c_x = int(M_y['m10']/M_y['m00'])
            c_y = int(M_y['m01']/M_y['m00'])
        else:
            self.pub.publish(self.twist)
            cv.imshow('HSV',hsv1)
            cv.waitKey(20)
            return 
            
        err = c_x - width/2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err)/500.0
        self.pub.publish(self.twist)

        hsv1 = cv.circle(hsv1, (c_x,c_y), 10, 0, -1)
        cv.imshow('HSV',hsv1)
        cv.waitKey(20)


if __name__=='__main__':
    rospy.init_node('follower_blue')
    follower = Follower()
    rospy.spin()