#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from waypointHelper import getTwist
from field import field
from field import point
from field import obstacle
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from zenith_obstacle_detector.msg import ObstacleList

rospy.init_node('zenith_path_planner')
#pub = rospy.Publisher('zenith/cmd_vel', Twist, queue_size = 1)
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 1)
currWay = 1
waypoints = [point(5,1),point(4,10),point(6,10),point(5,1)]
course = field(point(0,0),waypoints[currWay])

course.height = 11
course.width = 11
cost = 100
potFlag = 0
j=0
obstaclec = 1
obstaclek = 1.1
robotPose = Pose2D()
course.obstacles.append(obstacle(obstaclec,obstaclek,4,4))


def poseCallback(pose):
    global currWay
    global pub
    global j
    global cost
    global potFlag
    robotPose = pose
    dtw = course.distance(pose.x,pose.y,waypoints[currWay].x,waypoints[currWay].y)
    dtpw = course.distance(pose.x,pose.y,waypoints[currWay-1].x,waypoints[currWay-1].y)
    #print(dtpw > 1)
    #print(dtpw)
    if (dtw > 1):

        #print("twist sending")
        currcost = course.cost(pose.x,pose.y)
        futcost = course.cost(pose.x + (math.cos(pose.theta) * .5 ),pose.y + (math.sin(pose.theta) * .5 ))
        #print("curr: " + str(currcost) + ', fut: ' + str(futcost) + ', diff: ' + str(futcost - currcost))
        if(((currcost >= futcost) and not potFlag) or dtpw < 1):

            #print("Using waypoint")
            pub.publish(getTwist(pose,waypoints[currWay].x,waypoints[currWay].y))
            cost = course.cost(pose.x,pose.y)
        elif dtpw > 1:
            if(not potFlag):
                print("Using potential fields!")
            course.newPath(pose.x,pose.y)
            #print("len of new path: " + str(len(course.pathToCurrGoal)))
            #print(course.pathToCurrGoal)
            potFlag = 1
            if(course.distance(pose.x,pose.y,course.pathToCurrGoal[0][j],course.pathToCurrGoal[1][j]) > .2):
                #print ("Potential Field Point: (" + str(course.pathToCurrGoal[0][j]) + ', ' + str(course.pathToCurrGoal[1][j]) + ')')
                pub.publish(getTwist(pose,course.pathToCurrGoal[0][j],course.pathToCurrGoal[1][j]))
            elif(j == len(course.pathToCurrGoal) - 1):
                potFlag = 0
            else:
                j = j +1
    else:
        stopMsg = Twist()
        stopMsg.linear.x = 0
        stopMsg.angular.z = 0
        currWay = currWay + 1
        course.goals.append(waypoints[currWay])
        course.goals.pop(0)
        print("got there")
        print("new Goal")
        print(dtw)
        print ("(" + str(course.goals[0].x) + ', ' + str(course.goals[0].y) + ')')
        print ("(" + str(waypoints[currWay].x) + ', ' + str(waypoints[currWay].y) + ')')
        potFlag = 0

        pub.publish(stopMsg)

def obscallback(obslist):
    #print(obslist)
    obslist = obslist.obstacles
    #print(str(obslist))
    for i in range(0,len(obslist)):
        isFound = False
        xcoord = math.cos(robotPose.theta)*obslist[i].x - math.sin(robotPose.theta)*obslist[i].y + robotPose.x
        ycoord = -math.sin(robotPose.theta)*obslist[i].x + math.cos(robotPose.theta)*obslist[i].y + robotPose.y
        #print ("(" + str(xcoord) + ', ' + str(ycoord) + ')')

        if((xcoord > 0 and xcoord < course.height) and (ycoord < course.width/2 and ycoord > -course.width/2)):
            #print("obs in world frame " + str(course.height) + 'x' + str(course.width))
            for curobs in course.obstacles:
                dist = course.distance(xcoord, ycoord , curobs.x, curobs.y)
                #print(dist)
                if(dist < .8 and obslist[i].type == 'static'):
                    if(not isFound):
                        isFound = True
                        #print("obs already detected")
                        curobs.x = obslist[i].x
                        curobs.y = obslist[i].y
                    else:
                        #print("duplicate found")
                        course.obstacles.remove(curobs)

            if(not isFound and obslist[i].type == 'static'):
                print("new obstacle added!")
                print(obslist[i].type)
                print ("(" + str(xcoord) + ', ' + str(ycoord) + ')')
                course.obstacles.append(obstacle(obstaclec,obstaclek,xcoord,ycoord))



'''
    for i in range(0,len(obslist)):
        if(obslist[i].type == 'static'):
            print("found static obs")
            for j in range(0,len(course.obstacles)):
                xcoord = math.cos(pose.theta)*obslist[i].x - math.sin(pose.theta)*obslist[i].y + pose.x
                ycoord = math.sin(pose.theta)*obslist[i].x - math.cos(pose.theta)*obslist[i].y + pose.y
                if((xcoord > 0 and xcoord < course.height) and (ycoord < course.width/2 and ycoord > -course.width/2)):
                    print("obs in course")
                    dist = course.distance(xcoord, ycoord , course.obstacles[j].x, course.obstacles[j].y)
                    if(dist > .8 and obsobslist[i].type == 'static'):
                        print("new obstacle")
                        course.obstacles.append(obstacle(obstaclec,obstaclek,xcoord,ycoord))
                        break
        elif(obslist[i].type == 'moving'):
            stopMsg = Twist()
            stopMsg.linear.x = 0
            stopMsg.angular.z = 0
            pub.publish(stopMsg)
            '''
    #    if(obslist[i].type == 'static'):
            #course.obstacles.append(obstacle(obstaclec,obstaclek,,))
    #    if(obslist[i].type == 'moving'):

def listener():
    #rospy.Subscriber("zenith/pose2D", Pose2D, poseCallback,queue_size=1)
    rospy.Subscriber("turtle1/pose", Pose, poseCallback,queue_size=1)
    rospy.Subscriber("zenith/obstacles",ObstacleList,obscallback,queue_size=3)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
