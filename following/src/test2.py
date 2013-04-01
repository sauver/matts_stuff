#!/usr/bin/env python
import roslib
roslib.load_manifest('opencv_test')
import sys
import rospy
import cv
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    # set window name
    self.windowName = "Object Detection"
    #set thickness of outline
    self.strokeWeight = 1
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)

    
    self.cascade = cv2.CascadeClassifier('./cascade.xml')
   

  def callback(self,data):
    try:
      cv_img = self.bridge.imgmsg_to_cv(data, "bgr8")
      cv_image = numpy.fromstring(cv_img.tostring(),dtype=numpy.float32)
    except CvBridgeError, e:
      print e

    
        
    '''rects = self.cascade.detectMultiScale(cv_image, rejectLevels=1, levelWeights=1)
    if rects:
        #print 'face detected!'
        for x,y, width,height in rects:
            cv2.rectangle(img, (x,y), (x+width, y+height), color, strokeWeight)

    # display!'''
    cv2.imshow(self.windowName, cv.fromarray(cv_image,da[::2,::2,::-1]))
            

   

    try:
      self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
 
if __name__ == '__main__':
    main(sys.argv)



