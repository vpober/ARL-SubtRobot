#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def cmd_vel_test():
  pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
  rospy.init_node('cmd_vel_test', anonymous = True)
  vel_msg = Twist()
  rate = rospy.Rate(5)

  vel_msg.linear.x = 0.5
  vel_msg.linear.y = 0
  vel_msg.linear.z = 0
  vel_msg.angular.x = 0
  vel_msg.angular.y = 0
  vel_msg.angular.z = 0

  while not rospy.is_shutdown():
      pub.publish(vel_msg)
      if(vel_msg.linear.x):
          vel_msg.linear.x = 0
      else:
          vel_msg.linear.x = 0.5
      rate.sleep()

if __name__ == '__main__':
    try:
        cmd_vel_test()
    except rospy.ROSInterruptException: pass
