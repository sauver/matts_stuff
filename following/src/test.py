#!/usr/bin/env python
import roslib
roslib.load_manifest('opencv_test')
import sys
import rospy
import cv
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)
    self.pix_pub = rospy.Publisher("drone_pose", Pose)

    # create storage
    self.storage = cv.CreateMemStorage(0)
    self.cascade = cv.Load('parrot4.xml')
    

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e

    (cols,rows) = cv.GetSize(cv_image)
        
    faces = cv.HaarDetectObjects(cv_image, self.cascade, self.storage, 1.2, 5, 1, (cols/16, rows/16))
    tmpx = 0
    tmpy = 0
    pose = Pose()
    #cv.Circle(cv_image, (640/2, 480/2), 40, cv.CV_RGB(255, 255, 255)) 
    if faces:
        #print 'face detected!'
        for i in faces:
            
            cv.Rectangle(cv_image,(i[0][0], i[0][1]), (i[0][0]+i[0][2], i[0][1]+i[0][3]), 256)
            if i[0][0]+(i[0][2]/2) > tmpx and i[0][1]+(i[0][3]/2) > tmpy:
                tmpx = i[0][0]+(i[0][2]/2)
                tmpy = i[0][1]+(i[0][3]/2)
    if tmpx != 0 and tmpy !=0:
        pose.position.x = tmpx;
        pose.position.y = tmpy;
        self.pix_pub.publish(pose)

    else:
        #This will make the drone hover in place
        pose.position.x = 640/2
        pose.position.y = 480/2
        self.pix_pub.publish(pose)
    cv.ShowImage("Image window", cv_image)
    cv.WaitKey(3)

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
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
