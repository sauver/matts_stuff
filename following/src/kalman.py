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
    self.kalman_pub = rospy.Publisher("/kalman_pose", Point)
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

    self.z = np.array([300, 400, 0])
    #Might add update from yaw of drone (need to figure out difference between
    #that and the UGV frame
    #self. update_nav = False
    self.update_unfilt = False
    self.update_filt = False
    self.altd = 0


  def navdata_callback(self, data):
    self.altd = data.altd *1e-6 #m

  def unfilt_callback(self,data):
    #dont think this is right. Should probably be 1x9. Look into adding accleration to this as well
    self.z = np.array([data.x, data.y, data.z])
    self.update_filt = True


  def state_integrate(self):
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
      rate.sleep()
      #For integration model
      Q_k = np.eye(9)*.2
      w = np.random.multivariate_normal(np.zeros([1,9])[0], Q_k)
      #For sensor measurement
      R_k = np.eye(3)*.01
      v = np.random.multivariate_normal(np.zeros([1,3])[0], R_k)
      #Predict
      s_k =  np.transpose(np.dot(self.A, self.s))
      P_k = np.add(np.dot(np.dot(self.A, self.P), np.transpose(self.A)), Q_k)
      #Update
      if self.update_unfilt:
        y_k = np.subtract(np.transpose(np.add(self.z, v)), np.dot(self.H, s_k))
        S_K = np.dot(np.dot(self.H, P_k), np.transpose(self.H))
        K = np.dot(P_k, np.dot(np.transpose(self.H), np.pinv(S_K)))
        self.s = np.add(s_k, np.dot(K, y_k))
        self.P = np.dot(np.subtract(np.eye(9), np.dot(K, self.H)), P_k)
        #This could result in a terrible race condition. But will need to test
        self.update_filt = False
      else:
        self.s = s_k
        self.P = P_k



def main(args):
  kalman = simple_kalman()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"


if __name__ == '__main__':
  main(sys.argv)
