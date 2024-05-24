#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

def find_devices():
    for i in range(5):
        cap_test = cv2.VideoCapture(i)
        if cap_test.isOpened():
            print("Device {} is available.".format(i))
        cap_test.release()

def main():
    rospy.init_node('camera_node', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    find_devices()

    device_id = rospy.get_param("/camera/deviceID", 0)
    width = rospy.get_param("/camera/width", 640)
    height = rospy.get_param("/camera/height", 480)

    cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        rospy.logerr("Failed to open web cam")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    print("Set frame width:", width)

    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    print("Set frame height:", height)

    print("Current camera properties:")
    print("Frame width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print("Frame height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("Frame rate:", cap.get(cv2.CAP_PROP_FPS))

    bridge = CvBridge()
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            try:
                image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr(e)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
