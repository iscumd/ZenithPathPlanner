#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

rospy.init_node('zenith_path_planner')
pub = rospy.Publisher('zenith/cmd_vel', Twist,queue_size = 1)

Xwaypoints = [0,0]
Ywaypoints = [13,0]

def chop(x, xmin, xmax):
    if x < xmin:
        return xmin
    elif x > xmax:
        return xmax
    else:
        return x

def distance(x1,y1,x2,y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def angle_diff(a0, a1):
    return (((a0 - a1) + 3 * math.pi) % (2 * math.pi)) - math.pi


def getTwist(pose,newx,newy):
    linearTune = 10.0
    angularTune = 1.0

    d = distance(pose.x, pose.y,newx,newy)

    heading_to_p = math.atan2(newy - pose.y, newx - pose.x)
    print("Heading to Point: " + str(heading_to_p))
    heading_error = angle_diff(pose.theta, heading_to_p)
    print("Error: " + str(heading_error))
    w = chop(angularTune * heading_error, -0.16, 0.16)
    v = chop(1.0 / (linearTune * abs(heading_error)), 0.0, .5)

    dpose = Twist()
    dpose.linear.x = v
    dpose.angular.z = w

    return dpose

def callback(pose):
    #print("pose recieved")
    i = 0
    if (distance(pose.x,pose.y,Xwaypoints[i],Ywaypoints[i]) > .5):
        #print("twist sending")
        pub.publish(getTwist(pose,Xwaypoints[i],Ywaypoints[i]))
    else:
        if i < len(Xwaypoints):
            i = i + 1
        else:
            stopMsg = Twist()
            stopMsg.linear.x = 0
            stopMsg.angular.z = 0
            pub.publish(stopMsg)

def listener():
    rospy.Subscriber("zenith/pose2D", Pose2D, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
