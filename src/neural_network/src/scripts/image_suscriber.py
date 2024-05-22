#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def main_process(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    equalized_image = cv2.equalizelist(gray_image)
    cv2.imshow("Equalized image P1", equalized_image)
    cv2.waitKey(1)

def imageCallback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        cv2.imshow("Image Subscriber PY",cv_image)
        cv2.waitKey(1)
        main_process(cv_image)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s",e)

if __name__ == "__main__":
    rospy.init_node("image_subscriber_node_py")
    rospy.Subscriber("/camera/image_raw", Image, imageCallback)
    rospy.spin()
