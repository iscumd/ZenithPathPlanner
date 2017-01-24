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
from zenith_obstacle_detector.msg import ObstacleList

rospy.init_node('zenith_path_planner')
pub = rospy.Publisher('zenith/cmd_vel', Twist, queue_size = 1)
currWay = 0
waypoints = [point(13,0),point(0,0)]
course = field(point(0,0),waypoints[currWay])
course.height = 15
course.width = 4
cost = 100
potFlag = 0
j=0
obstaclec = 2
obstaclek = 1
robotPose = Pose2D()


def poseCallback(pose):
    global currWay
    global pub
    global j
    global cost
    global potFlag
    robotPose = pose
    if (course.distance(pose.x,pose.y,waypoints[0].x,waypoints[0].y) > .3):

        #print("twist sending")
        currcost = course.cost(pose.x,pose.y)
        futcost = course.cost(pose.x + (math.cos(pose.theta) * .01 ),pose.y + (math.sin(pose.theta) * .01 ))
        #print("curr: " + str(currcost) + ', fut: ' + str(futcost) + ', diff: ' + str(futcost - currcost))
        if(course.cost(pose.x,pose.y) >= course.cost(pose.x + (math.cos(pose.theta) * .01 ),pose.y + (math.sin(pose.theta) * .01 )) and not potFlag):

            print("Using waypoint")
            pub.publish(getTwist(pose,waypoints[0].x,waypoints[0].y))
            cost = course.cost(pose.x,pose.y)
        else:
            print("Using potential fields!")
            course.newPath(pose.x,pose.y)
            #print("len of new path: " + str(len(course.pathToCurrGoal)))
            print(course.pathToCurrGoal)
            potFlag = 1
            if(course.distance(pose.x,pose.y,course.pathToCurrGoal[0][j],course.pathToCurrGoal[1][j]) > .2):
                print ("Potential Field Point: (" + str(course.pathToCurrGoal[0][j]) + ', ' + str(course.pathToCurrGoal[1][j]) + ')')
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
        course.goals.pop(0)
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
    rospy.Subscriber("zenith/pose2D", Pose2D, poseCallback,queue_size=1)
    rospy.Subscriber("zenith/obstacles",ObstacleList,obscallback,queue_size=3)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
