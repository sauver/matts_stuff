#!/usr/bin/env python
import roslib
roslib.load_manifest('following')
import sys
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point, PoseArray
from ardrone2.msg import Navdata
from sensor_msgs.msg import Joy

class simple_kalman:

  def __init__(self):
    rospy.init_node('height_detect', anonymous=True)
    self.drone_sub = rospy.Subscriber("ardrone/navdata",Navdata,self.navdata_callback)
    self.pose_sub = rospy.Subscriber("uav/pose",Point,self.unfilt_callback)
    self.twist_pub = rospy.Publisher("/kalman_poise", Point)
    #Assume Constant Acceleration assumption for x, y, theta
    self.A = np.array([[1, 0, 0, 1, 0, 0, .5, 0, 0],/
                       [0, 1, 0, 0, 1, 0, 0, .5, 0],/
                       [0, 0, 1, 0, 0, 1, 0, 0, .5],/
                       [0, 0, 0, 1, 0, 0, 1, 0, 0],/
                       [0, 0, 0, 0, 1, 0, 0, 1, 0],/
                       [0, 0, 0, 0, 0, 1, 0, 0, 1],/
                       [0, 0, 0, 0, 0, 0, 1, 0, 0],/
                       [0, 0, 0, 0, 0, 0, 0, 1, 0],/
                       [0, 0, 0, 0, 0, 0, 0, 0, 1]])
    #Updates just update position and heading
    self.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],/
                       [0, 1, 0, 0, 0, 0, 0, 0, 0],/
                       [0, 0, 1, 0, 0, 0, 0, 0, 0]])
    self.P = np.eye(9)*.1
    #Assume drone starts in the center of the 600x800 image with zero heading
    # [x, y, theta, x_dot, y_dot, theta_dot, x_dd, y_dd, theta_dd]
    self.s = np.array([300, 400,0, 0, 0, 0, 0, 0, 0])

    #Might add update from yaw of drone (need to figure out difference between
    #that and the UGV frame
    #self. update_nav = False
    self.update_unfilt = False
    self.update_filt = True
    self.altd = 0


  def navdata_callback(self, data):
    self.altd = data.altd *1e-6 #m

  def unfilt_callback(self,data):
    pass


  def state_integrate(self):
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
      rate.sleep()
      Q_k = np.eye(9)*.2
      w = np.random.multivarate_normal(np.zeros([1,9])[0], Q_k)
      s_k =  np.dot(self.A, self.s)
      P_k = np.add(np.dot(np.dot(self.A, self.P), np.transpose(self.A)), Q_k)



def main(args):
  kalman = simple_kalman()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"


if __name__ == '__main__':
  main(sys.argv)
