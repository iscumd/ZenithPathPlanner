from math import pi
import rospy
from geometry_msgs.msg import Twist


rospy.init_node('zenith_path_planner')
pub = rospy.Publisher('zenith/cmd_vel', Twist, queue_size = 1)
dpose = Twist()
dpose.linear.x = 0
dpose.angular.z = -.7
while(True):
    pub.publish(dpose)
