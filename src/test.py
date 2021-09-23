#!/usr/bin/env python
import rospy

import os
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

img_msg_name="/camera/color/image_raw"
depth_img_msg_name="/camera/depth/image_rect_raw"
# img_msg_name="/darknet_ros/detection_image"

width = 640
height = 480
indexes = []

def depth_imageCallback(msg):
    global indexes
    now_indexes  = indexes[:]
    print("Depth - CALLBACK")

    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg)

    img = cv2.resize(img, dsize=(width, height))

    re_img = np.zeros((480, 640), dtype=np.uint8)
    # img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)


    for i,a in enumerate(now_indexes):
        # print("{} |||| {}, {}VAL------------{} ".format(i, a%width,a//width,img.item(a//width,a%width)))
        re_img.itemset(a//width, a%width, 200)

    # for y in range(height):
    #     list = []
    #     for x in range(width):
    #         list.append(img.item(y,x)//10)
            # print(img.item(y,x), end=' ')
        # print(list)
    
    cv2.imshow("depth_img",img*50)   
    cv2.imshow("depth_re_img",re_img)   
    cv2.waitKey(1)

    os.system('clear')

def imageCallback(msg):
    global indexes
    indexes = []
    print("CALLBACK")

    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg)

    img = cv2.resize(img, dsize=(width, height))
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    # cv2.imshow("raw_img",img)   



    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([25, 255, 255])
    
    mask = cv2.inRange(img, lower_red, upper_red)
    res = cv2.bitwise_and(img,img, mask= mask)
    cv2.imshow("filtered_image",res)

    idx = 0
    full =0
    for y in range(height):
        for x in range(width):
            val = res.item(y,x,0)
            full+=1
            if val is not 0:
                idx +=1
                indexes.append(x + y*width)

    # print(idx)
    # print(full)

def main():
    rospy.init_node("TF_test")
    rospy.Subscriber(img_msg_name,Image, imageCallback, queue_size=1)
    rospy.Subscriber(depth_img_msg_name,Image, depth_imageCallback, queue_size=1)


    print("SDFSDF")
    print("SDFSDF")
    print("SDFSDF")
    print("SDFSDF")
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        cv2.waitKey(1)
        rate.sleep()

    return
if __name__=="__main__":
    main()