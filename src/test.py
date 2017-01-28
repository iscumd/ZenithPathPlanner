#!/usr/bin/env python
# license removed for brevity
import rospy
from waypointHelper import distance
from waypointHelper import getTwist
from waypointHelper import point
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from zenith_obstacle_detector.msg import ObstacleList

rospy.init_node('zenith_path_planner')
pub = rospy.Publisher('zenith/cmd_vel', Twist, queue_size = 1)

waypoint = [point(4,0),point(0,0)]
currWay = 0
backUpMode = 1

def poseCallback(pose):
    global backUpMode
    global currWay
    global waypoint
    modTwist = Twist()
    dtw = distance(pose.x,pose.y,waypoint[currWay].x,waypoint[currWay].y) #distance to waypoint
    if(dtw > 0.2):
        modTwist = getTwist(pose,waypoint[currWay].x,waypoint[currWay].y,backUpMode)
        pub.publish(modTwist)

    else:
        currWay = currWay + 1
        backUpMode = -1

def listener():
    rospy.Subscriber("zenith/pose2D", Pose2D, poseCallback,queue_size=1)
    #rospy.Subscriber("zenith/pose2D", Pose, poseCallback,queue_size=1)
    #rospy.Subscriber("zenith/obstacles",ObstacleList,obscallback,queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
