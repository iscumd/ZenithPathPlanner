#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from waypointHelper import getTwist
from field import field
from field import point
from field import obstacle
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

rospy.init_node('zenith_path_planner')
pub = rospy.Publisher('zenith/cmd_vel', Twist, queue_size = 1)
currWay = 0
waypoints = [point(13,0),point(0,0)]
course = field(point(0,0),waypoints[currWay])
cost = 100
potFlag = 0
j=0
obstaclec = 1
obstaclek = 1
robotPose

def poseCallback(pose):
    global currWay
    global pub
    global j
    robotPose = pose
    if (distance(pose.x,pose.y,waypoint.x,waypoint.y) > .3):
        #print("twist sending")
        if(course.cost(pose.x,pose.y)-cost <= 0 and not potFlag)
            pub.publish(getTwist(pose,waypoint.x,waypoint.y))
            cost = course.cost(pose.x,pose.y)
        else:
            course.newPath()
            potFlag = 1
            if(distance(pose.x,pose.y,pathToCurrGoal[j].x,pathToCurrGoal[j].y) > .2):
                pub.publish(getTwist(pose,pathToCurrGoal[j].x,pathToCurrGoal[j].y))
            elif(j == len(course.pathToCurrGoal) - 1):
                potFlag = 0
            else:
                j++
    else:
        stopMsg = Twist()
        stopMsg.linear.x = 0
        stopMsg.angular.z = 0
        currWay = currWay + 1
        course.goals.pop(0)
        pub.publish(stopMsg)

def obscallback(list):
    for i in range(0,len(list)):
        if(list[i].type == 'static'):
            for j in range(0,len(course.obstacles)):
                xcoord = math.cos(pose.theta)*list[i].x - math.sin(pose.theta)*list[i].y + pose.x
                ycoord = math.sin(pose.theta)*list[i].x - math.cos(pose.theta)*list[i].y + pose.y
                if((xcoord > 0 and xcoord < course.height) and (ycoord < course.width/2 and ycoord > -course.width/2)):
                    dist = course.distance(xcoord, ycoord , course.obstacles[j].x, course.obstacles[j].y)
                    if(dist > .8 and list[i].type == 'static'):
                        course.obstacles.append(obstacle(obstaclec,obstaclek,xcoord,ycoord)):
                        break
        elif(list[i].type == 'moving'):
            stopMsg = Twist()
            stopMsg.linear.x = 0
            stopMsg.angular.z = 0
            pub.publish(stopMsg)
    #    if(list[i].type == 'static'):
            #course.obstacles.append(obstacle(obstaclec,obstaclek,,))
    #    if(list[i].type == 'moving'):

def listener():
    rospy.Subscriber("zenith/pose2D", Pose2D, poseCallback)
    rospy.Subscriber("zenith/obstacles",custom,obscallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
