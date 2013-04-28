#!/usr/bin/env python
import roslib
roslib.load_manifest('following')
import sys
import math
import rospy
from geometry_msgs.msg import Twist, Point
from ardrone2.msg import Navdata
from sensor_msgs.msg import Joy

class follower:

  def __init__(self):
    rospy.init_node('follower', anonymous=True)
    self.twist_pub = rospy.Publisher("/cmd_vel", Twist)
    self.pose_sub = rospy.Subscriber("uav/pose",Point,self.callback)
    #self.drone_sub = rospy.Subscriber("ardrone/navdata",Navdata,self.navdata_callback)
    self.drone_sub = rospy.Subscriber("/height",Point,self.navdata_callback)
    self.joy_sub = rospy.Subscriber("/joy",Joy,self.joy_callback)
    stop = Twist()
    stop.linear.x = 0
    stop.linear.y = 0
    stop.linear.z = 0
    stop.angular.z = 0
    self.twist_pub.publish(stop)

    self.altd = 0
    self.x_prev = 0
    self.y_prev = 0
    self.rot_set = 0

  def joy_callback(self, data):
    y_axis = data.axes[3]
    self.rot_set = y_axis*140 #Turn a max of 140 degrees in each direction. This is arbitrary

  def navdata_callback(self, data):
    #self.altd = data.altd *1e-6 #m
    self.altd = data.z #m

  def callback(self,data):
    twist = Twist()
    x = data.x
    y = data.y

    angle = -data.z
    if x == 0 and y== 0:
      height = 2
    else:
      height = 2

    x_diff = ((600/2)-x)/(600/2)
    y_diff = (y-(800/2))/(800/2)
    altd_diff = (height-self.altd) #keep the uav at a fixed 2m height

    #TODO Make sure this subtraction is correct
    z_cmd = (self.rot_set - angle)*3/180 #Tank Following
    #z_cmd = (self.rot_set - angle)*2.5/180 #Tank Following

    #PD controller
    x_cmd = x_diff*.08 + (x - self.x_prev)*.002
    y_cmd = y_diff*.08 + (y - self.y_prev)*.002

    #x_cmd = x_cmd*math.cos(math.pi*angle/180)
    #y_cmd = y_cmd*math.sin(math.pi*angle/180)


    altd_cmd = altd_diff*.8
    detect = True
    #Angle outside of range to command x and y. Just control yaw
    if angle > 10 or angle < -10:
      if abs(z_cmd) > .2:
        z_cmd = .2*cmp(z_cmd,0)

        x_cmd = 0
        y_cmd = 0
    #Angle in range to control x and y
    else:
      if abs(z_cmd) > .3:
        z_cmd = .3*cmp(z_cmd,0)

      if abs(x_cmd) > .04:
        x_cmd = .04*cmp(x_cmd,0)

      if abs(y_cmd) > .04:
        y_cmd = .04*cmp(y_cmd,0)

      if abs(altd_cmd) > 1.4:
        altd_cmd = 1.4*cmp(altd_cmd,0)

      if x == 0 and y == 0:
        x_cmd = 0
        y_cmd = 0
        detect = False

    #print abs(x_diff), abs(y_diff), x_cmd, y_cmd, z_cmd, '#######', altd_cmd, '######',  self.altd, detect
    if abs(x_diff) > 0.2:
      twist.linear.x = x_cmd
    else:
      twist.linear.x = 0
    if abs(y_diff) > 0.2:
      twist.linear.y = y_cmd
    else:
      twist.linear.y = 0
    #twist.linear.z = altd_cmd
    twist.linear.z = 0
    twist.angular.z = z_cmd
    print twist
    self.twist_pub.publish(twist)
    self.x_prev = x
    self.y_prev = y


def main(args):
  follow = follower()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"


if __name__ == '__main__':
  main(sys.argv)
