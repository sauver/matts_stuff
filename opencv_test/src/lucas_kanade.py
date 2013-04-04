#!/usr/bin/env python
import roslib
roslib.load_manifest('opencv_test')
import sys
import rospy
import cv
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)

    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("network/image_raw",Image,self.callback)


  def callback(self,data):
    try:
      img = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e

    gradientX =  cv.CreateMat(img.width, img.height, cv.CV_16S)
    kern = cv.CreateMat(2,2, cv.CV_32FC1)
    new_img =  cv.CreateImage(cv.GetSize(img), 8, 1)
    cv.CvtColor(img, new_img, cv.CV_BGR2GRAY)
    cv.Set(kern,0)
    kern[0,0] = 1
    kern[0,1] = -1
    print kern
    cv.Filter2D(new_img,gradientX,kern)
    cv.ShowImage("Image window", gradientX)
    cv.WaitKey(3)

def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
