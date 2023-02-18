#!/usr/bin/env python3

#  line above is needed so this is executed as a python script

import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge

def image_callback(msg): 
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg) # "bgr8")
    # (rows, cols, channels) = cv_image.shape
    print(cv_image[0].shape)
   
    # cv.imshow("Image", cv_image[0])
    # cv.waitKey(0)
    # rospy.loginfo(msg)

if __name__ == "__main__": 
    rospy. init_node("raw_image_subscriber")

    sub = rospy.Subscriber("/panda_camera/depth/image_raw", Image, callback=image_callback)

    rospy.loginfo("Node has been started.")

    try: 
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

    # cv.destroyAllWindows()

