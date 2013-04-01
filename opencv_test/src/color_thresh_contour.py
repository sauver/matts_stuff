#!/usr/bin/env python
import roslib
roslib.load_manifest('opencv_test')
import sys
import math
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)

    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("network/image_raw",Image,self.callback)
    self.pos_pub = rospy.Publisher("uav/pose", Point)

  def callback(self,data):
    try:
      img = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e



#blur the source image to reduce color noise e
    cv.Smooth(img, img, cv.CV_BLUR, 3);

#convert the image to hsv(Hue, Saturation, Value) so its
#easier to determine the color to track(hue)
    hsv_img = cv.CreateImage(cv.GetSize(img), 8, 3)
    cv.CvtColor(img, hsv_img, cv.CV_BGR2HSV)

#limit all pixels that don't match our criteria, in this case we are
#looking for purple but if you want you can adjust the first value in
#both turples which is the hue range(120,140).  OpenCV uses 0-180 as
#a hue range for the HSV color model
    thresholded_img =  cv.CreateImage(cv.GetSize(hsv_img), 8, 1)
    thresholded_img_y =  cv.CreateImage(cv.GetSize(hsv_img), 8, 1)
    cv.InRangeS(hsv_img, (160, 75, 60), (180, 255, 255), thresholded_img) #red
    cv.InRangeS(hsv_img, (20, 60, 60), (40, 255, 255), thresholded_img_y) #yellow

#determine the objects moments and check that the area is large
#enough to be our object
    mat=cv.GetMat(thresholded_img)
    mat_y=cv.GetMat(thresholded_img_y)
    cv.Erode(thresholded_img_y,thresholded_img_y,None,3)
    cv.Dilate(thresholded_img_y, thresholded_img_y,None,4)
    cv.Erode(thresholded_img, thresholded_img,None,3)
    cv.Dilate(thresholded_img, thresholded_img,None,4)

    moments = cv.Moments(mat, 0)
    area = cv.GetCentralMoment(moments, 0, 0)
    moments_y = cv.Moments(mat_y, 0)
    area_y = cv.GetCentralMoment(moments_y, 0, 0)
    uav_pos = Point()
#there can be noise in the video so ignore objects with small areas
    if(area > 10000) and (area_y > 10000):
        x = cv.GetSpatialMoment(moments, 1, 0)/area
        y = cv.GetSpatialMoment(moments, 0, 1)/area
        x_y = cv.GetSpatialMoment(moments_y, 1, 0)/area_y
        y_y = cv.GetSpatialMoment(moments_y, 0, 1)/area_y

        angle = math.atan2(x - x_y, y - y_y)*180/math.pi
        #X and y are flipped to put it into the robot frame
        uav_x = (y+y_y)/2
        uav_y =  (x+x_y)/2
        uav_pos.x = uav_x
        uav_pos.y = uav_y
        uav_pos.z = angle
        #flip x and y to put it in the UAV's frame
      	cv.Circle(img,(int(uav_y), int(uav_x)),5,(0,255,255))
        cv.Line(img,(int(uav_y), int(uav_x)),(int(uav_y - 20*math.sin(angle*math.pi/180)), int(uav_x - 20*math.cos(angle*math.pi/180))),(0,255,255),3,8,0)
    self.pos_pub.publish(uav_pos)
    added = cv.CreateImage(cv.GetSize(img), 8, 1)
    cv.Add(thresholded_img, thresholded_img_y, added)
    cv.ShowImage("Image window", img)
    cv.WaitKey(3)
    del(thresholded_img)
    del(thresholded_img_y)
    del(added)
    del(mat)
    del(mat_y)
    del(hsv_img)
    del(img)

def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
