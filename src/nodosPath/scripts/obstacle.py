#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def wl_callback(data):
    global wl
    wl = data.data


if __name__=="__main__":
    try:
        rospy.init_node('mushu_face')
        nodeRate = 100
        rate = rospy.Rate(nodeRate)