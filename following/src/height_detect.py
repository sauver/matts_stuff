#!/usr/bin/env python
import roslib
roslib.load_manifest('following')
import sys
import math
import rospy
from geometry_msgs.msg import Twist, Point, PoseArray
from ardrone2.msg import Navdata
from sensor_msgs.msg import Joy

class height_detect:

  def __init__(self):
    rospy.init_node('height_detect', anonymous=True)
    self.drone_sub = rospy.Subscriber("ardrone/navdata",Navdata,self.navdata_callback)
    self.pose_sub = rospy.Subscriber("uav/centers",PoseArray,self.callback)

    self.altd = 0


  def navdata_callback(self, data):
    self.altd = data.altd *1e-6 #m

  def callback(self,data):
    if data.poses != []:

      rat_height = .0024/600
      rat_wit = .0032/800

      red_x = data.poses[0].position.x
      red_y = data.poses[0].position.y
      yellow_x = data.poses[1].position.x
      yellow_y = data.poses[1].position.y

      x_dist = abs(red_x - yellow_x)*rat_height
      y_dist = abs(red_y - yellow_y)*rat_wit

      dist = math.sqrt(x_dist**2 + y_dist**2)

      f = 0.0028 #m focal length
      actual = .3 #m actual distance

      height = actual*f/dist

      print height

def main(args):
  height_det = height_detect()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"


if __name__ == '__main__':
  main(sys.argv)
