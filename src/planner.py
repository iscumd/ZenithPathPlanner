#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
currWay = 0
rospy.init_node('zenith_path_planner')
pub = rospy.Publisher('zenith/cmd_vel', Twist,queue_size = 1)

Xwaypoints = [13,0]
Ywaypoints = [0,0]

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


def getHeadingErr(actual,target): #input in radians
    pi = math.pi
    """
    Actual------Target-------Error-------Difference(t-a)
    0           2pi             0           2pi
    0           pi/4            pi/4        pi/4
    pi/4        0               -pi/4       -pi/4
    6.27        0               -0.01       -6.27
    0           6.27            0.01        6.27
    -pi/6       9pi/6       -2pi/6(-pi/3)   10pi/6
    """

    err = target-actual
    if (err > pi & target > 0 & actual > 0):
        err = 2*pi - (target + actual)
        return err
    elif (target < 0 & actual < 0):
        err = -1 * (target - actual)
        return err
    elif (target > 0 & actual < 0):
        err = (2*pi - target) - (-1*actual)
    elif (target < 0 & actual > 0):
        return 0



def getTwist(pose,newx,newy):
    linearTune = 10.0
    angularTune = 1.0

    d = distance(pose.x, pose.y,newx,newy)

    heading_to_p = math.atan2(newy - pose.y, newx - pose.x)
    print("Heading to Point: " + str(heading_to_p))
    heading_error = angle_diff(pose.theta, heading_to_p)
    print("Error: " + str(heading_error))
    w = chop(-angularTune * heading_error, -0.2, 0.2)
    v = chop(1.0 / (linearTune * abs(heading_error)), 0.0, .75)

    dpose = Twist()
    dpose.linear.x = v
    dpose.angular.z = w

    return dpose

def callback(pose):
    #print("pose recieved")
    global currWay

    if (distance(pose.x,pose.y,Xwaypoints[currWay],Ywaypoints[currWay]) > .5):
        #print("twist sending")
        pub.publish(getTwist(pose,Xwaypoints[currWay],Ywaypoints[currWay]))
    else:
        if currWay < len(Xwaypoints):
            currWay = currWay + 1
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
