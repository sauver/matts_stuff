#!/usr/bin/env python
import roslib
roslib.load_manifest('opencv_test')
import sys
import math
import rospy
import cv
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
class image_converter:

  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)

    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("network/image_raw",Image,self.callback)
    self.image_pub = rospy.Publisher("network/image_raw_filt",Image)

  def callback(self,data):
    try:
      img = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e

    imagesize = cv.GetSize(img)
    r =cv.CreateImage(imagesize,8,1)
    g =cv.CreateImage(imagesize,8,1)
    b =cv.CreateImage(imagesize,8,1)

    tmp =cv.CreateImage(imagesize,8,1)
    tmp2 =cv.CreateImage(imagesize,8,1)

    filt_r =cv.CreateImage(imagesize,8,1)
    avg_r =cv.CreateImage(imagesize,8,1)
    filt_g =cv.CreateImage(imagesize,8,1)
    avg_g =cv.CreateImage(imagesize,8,1)
    out =cv.CreateImage(imagesize,8,3)

    #Split image into brg
    cv.Split(img, b, g, r, None)

    #For Red remove green and blue values from each pixel
    cv.Sub( r, g, tmp)
    cv.Sub(r, b, tmp2)
    cv.Add(tmp, tmp2, filt_r)
    minVal_r, maxVal_r, minLoc_r, maxLoc_r = cv.MinMaxLoc(filt_r)
    mat = cv.CreateMat(imagesize[1], imagesize[0], cv.CV_32FC1)
    cv.Set(mat, 255/maxVal_r) #Normalize based on max
    cv.Mul(filt_r, mat, avg_r)

    #For green renive red and blue values from each pixel
    cv.Sub( g, r, tmp)
    cv.Sub(g, b, tmp2)
    cv.Add(tmp, tmp2, filt_g)
    minVal_g, maxVal_g, minLoc_g, maxLoc_g = cv.MinMaxLoc(filt_g)
    cv.Set(mat, 255/maxVal_g) #Normalize based on max
    cv.Mul(filt_g, mat, avg_g)
    #For blue set it to 0
    cv.Set(mat, 0)
    cv.Mul(b, mat, b)

    #Create new image in out if the UAV in in frame (based on max pixel values)
    if maxVal_r > 100 and maxVal_g > 100:
      cv.Merge(b, avg_g, avg_r, None , out)
    else:
      out = img

    cv.ShowImage("Image window", out)
    cv.WaitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv_to_imgmsg(out, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
