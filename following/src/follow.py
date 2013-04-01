#!/usr/bin/env python
import roslib
roslib.load_manifest('following')
import sys
import rospy
from geometry_msgs.msg import Pose, Twist


class follower:

	def __init__(self):
		rospy.init_node('follower', anonymous=True)    
		self.twist_pub = rospy.Publisher("/cmd_vel", Twist)
		self.image_sub = rospy.Subscriber("/drone_pose",Pose,self.callback)

    

	def callback(self,data):
		twist = Twist()
		x = data.position.x
		y = data.position.y
		x_diff = (x - (480/2))/(480/2)
		y_diff = (y - (640/2))/(640/2)
		
		x_cmd = x_diff*.04
		y_cmd = y_diff*.04

		twist.linear.x = x_cmd
		twist.linear.y = y_cmd
		
		self.twist_pub.publish(twist)		
		
		

def main(args):
	follow = follower()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
  

if __name__ == '__main__':
	main(sys.argv)
