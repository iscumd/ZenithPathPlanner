#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from waypointHelper import getTwist
from waypointHelper import distance
from waypointHelper import point
from waypointHelper import obstacle
from waypointHelper import box
from waypointHelper import distance
from waypointHelper import angle_diff
from waypointHelper import isInBox
from waypointHelper import turnAbit
from waypointHelper import pscale
from waypointHelper import kenTwist
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from zenith_obstacle_detector.msg import ObstacleList

course = box(15,4,0,0)
logicBox = box(11,2,7.5,0)
robotPose = Pose2D()
obstacles = []
currWay = 0
waypoint = [point(5,10),point(5,1)]
angularModifier = .5

rospy.init_node('zenith_path_planner')
#pub = rospy.Publisher('zenith/cmd_vel', Twist, queue_size = 1)
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 1)
obstacles.append(point(5,5))

def poseCallback(pose):
    global robotPose
    global obstacles
    global logicBox
    global currWay
    robotPose = pose
    modTwist = Twist()
    dtw = distance(pose.x,pose.y,waypoint[currWay].x,waypoint[currWay].y) #distance to waypoint
    if(dtw > 0.2):
        modTwist = getTwist(pose,waypoint[currWay].x,waypoint[currWay].y)
        if(len(obstacles) == 0):
            pub.publish(modTwist)
            #print("No obstacles to avoid! :D")
        else:
            for obs in obstacles:
                dto = distance(pose.x,pose.y,obs.x,obs.y) # distance to obstacle
                if(dto < 2.5):
                    htp = math.atan2(waypoint[currWay].y - pose.y, waypoint[currWay].x - pose.x) # heading to point
                    hto = math.atan2(obs.y - pose.y, obs.x - pose.x) # heading to obstacle
                    hdiff = angle_diff(hto,htp)
                    print(hdiff)

                    if(hdiff >= -math.pi/2 and hdiff <= math.pi/2):
                        modTwist = kenTwist(pose, hdiff, dto, hto)

                    pub.publish(modTwist)

                else:
                    pub.publish(modTwist)
                    #print("No obstacles to avoid! :D")
    else:
        currWay = currWay + 1

def obscallback(obsList):
    global robotPose
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
            for curobs in obstacles:
                dist = distance(xcoord, ycoord , curobs.x, curobs.y)
                #print(dist)
                if(dist < .8 and obslist[i].type == 'static'):
                    if(not isFound):
                        isFound = True
                        #print("obs already detected")
                        curobs.x = obslist[i].x
                        curobs.y = obslist[i].y
                    else:
                        #print("duplicate found")
                        obstacles.remove(curobs)

            if(not isFound and obslist[i].type == 'static'):
                print("new obstacle added!")
                print(obslist[i].type)
                print ("(" + str(xcoord) + ', ' + str(ycoord) + ')')
                obstacles.append(obstacle(obstaclec,obstaclek,xcoord,ycoord))

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
