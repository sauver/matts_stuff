#!/usr/bin/env python
import roslib
roslib.load_manifest('opencv_test')
import sys
import rospy
import cv
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



#blur the source image to reduce color noise
    cv.Smooth(img, img, cv.CV_BLUR, 3);

#convert the image to hsv(Hue, Saturation, Value) so its
#easier to determine the color to track(hue)
    hsv_img = cv.CreateImage(cv.GetSize(img), 8, 3)
    cv.CvtColor(img, hsv_img, cv.CV_BGR2HSV)
    del hsv_img
    cv.ShowImage("Image window", img)
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
