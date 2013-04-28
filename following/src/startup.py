#!/usr/bin/env python
import roslib
roslib.load_manifest('following')
import sys
import math
import rospy
import time
from std_msgs.msg import String

class autopilot_startup:

  def __init__(self):
    rospy.init_node('autopilot_startup', anonymous=True)
    self.startup_pub = rospy.Publisher("/tum_ardrone/com", String)
    self.write_autopilot()

  def write_autopilot(self):
    start_file = open('/home/matt/fuerte_workspace/matt_sauver/following/flightPlans/initDemo.txt', 'r')
    time.sleep(10)
    i = 0
    for line in start_file:
      line = "c " + line.strip('\r\n')
      print line
      self.startup_pub.publish(String(line))
      if i == 2 or i == 1:
        time.sleep(5)
      else:
        time.sleep(2)
      i += 1
def main(args):
  autopilot = autopilot_startup()


if __name__ == '__main__':
  main(sys.argv)
