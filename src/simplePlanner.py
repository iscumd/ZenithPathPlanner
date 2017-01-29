#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import time
from waypointHelper import chop
from waypointHelper import getTwist
from waypointHelper import distance
from waypointHelper import point
from waypointHelper import obstacle
from waypointHelper import box
from waypointHelper import distance
from waypointHelper import angle_diff
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from zenith_obstacle_detector.msg import ObstacleList

course = box(13,3,0,0)
logicBox = box(11,2,7.5,0)
robotPose = Pose2D()
robotPose.x = 0
robotPose.y = 0
robotPose.theta = 0
obstacles = []
currWay = 0
waypoint = [point(7.8,0),point(0,0)]
angularModifier = .5
firstPose = 0
rospy.init_node('zenith_path_planner')
pub = rospy.Publisher('zenith/cmd_vel', Twist, queue_size = 1)
stall = 0
stop = 0
stopMode = False
obsTime = 0
ignoreObs = 0
angVel = 0
linVel = 0
velTime = int(round(time.time() * 1000))

last_heading_err = 0
aErrAcc = 0
last_speed_err = 0
lErrAcc = 0

#pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 1)
#obstacles.append(point(5,5))

def getUnstuck():
    backwards = Twist()
    backwards.linear.x = -0.25
    backwards.angular.z = 0
    pub.publish(backwards)

def stop():
    global stopMode
    stopMode = True
    stopMsg = Twist()
    stopMsg.linear.x = 0
    stopMsg.angular.y = 0
    pub.publish(stopMsg)

def poseCallback(pose):
    global robotPose
    global obstacles
    global logicBox
    global currWay
    global waypoint
    global firstPose
    global stall
    global stopMode
    global velTime
    global linVel
    global angVel
    global last_heading_err
    global aErrAcc
    global last_speed_err
    global lErrAcc

    targetSpeed = 1
    linearTune = 10.0
    lp = 1 #linear Proportional
    li = 1 #linear integrator
    ld = 1 #linear differential
    ap = 1 #Formerly Angular tune
    ai = 1 #angular integrator
    ad = 1 #angular differential
    dir = 1 #1 is forward -1 is backwards

    #print(pose)
    #print(int(round(time.time() * 1000)))
    backUpMode = False
    firstPose = 1
    dflp = distance(pose.x,pose.y,robotPose.x,robotPose.y) #distance from last pose
    tc = angle_diff(pose.theta,robotPose.theta) #theta change
    #velocity calculations
    now = time.time()
    dt = ( now - velTime)
    velTime = now

    angVel = (pose.theta - robotPose.theta)/dt
    linVel = (pose.x - robotPose.x)/dt
    print 'lin: ' + str(linVel) + '   [][][][]   ang: ' + str(angVel) + ' [][][][]   dt: ' + str(dt)


    robotPose = pose
    modTwist = Twist()
    dtw = distance(pose.x,pose.y,waypoint[currWay].x,waypoint[currWay].y) #distance to waypoint

    if(dtw < 0.2):
        currWay = currWay + 1

    print(currWay)

    if(currWay < len(waypoint)-1):

        d = distance(pose.x, pose.y,waypoint[currWay].x,waypoint[currWay].y)

        heading_to_p = math.atan2(waypoint[currWay].y - pose.y, waypoint[currWay].x - pose.x)
        #print("Heading to Point: " + str(heading_to_p))
        if(dir == -1):
            heading_error = angle_diff( heading_to_p,angle_diff(pose.theta, math.pi))
            print("Reverse Mode")
        else:
            heading_error = angle_diff(heading_to_p, pose.theta)

        #aErrAcc is angular Error Accumulator
        aErrAcc = heading_error*dt + aErrAcc
        last_heading_err = heading_error
        aErrDif = (heading_error - last_heading_err)/dt
        prechopW = ap * heading_error + ai * aErrAcc + ad * aErrDif
        w = chop(prechopW, -1,1)

        #aErrAcc is angular Error Accumulator

        adjusted_Target_Speed = chop(targetSpeed / (linearTune * abs(heading_error) + .000000001), 0.0, targetSpeed)
        speed_error = linVel - adjusted_Target_Speed
        lErrAcc = speed_error*dt + lErrAcc
        last_speed_err = speed_error
        lErrDif = (speed_error - last_speed_err)/dt
        prechopV = lp * speed_error + li * lErrAcc + ld * lErrDif
        v = chop(prechopV, 0.0, .5)

        if(dir == -1):
            v = v*-1

        modTwist.linear.x = v
        modTwist.angular.z = w

        #modTwist = getTwist(pose,waypoint[currWay].x,waypoint[currWay].y,1)

    else:

       modTwist.linear.x = 0
       modTwist.angular.y = 0
       print("End of Path")

    #print stopMode
    if(not stopMode):
        pub.publish(modTwist)
    else:
        stop()

def obscallback(obslist):
    global robotPose
    global obstacles
    global firstPose
    global stop
    global obsTime
    global ignoreObs
    global stopMode
    stopSec = 5
    #print(obslist)
    obslist = obslist.obstacles
    #print(str(obslist))
    if(int(round(time.time() * 1000)) - obsTime > stopSec*1000):
        stopMode = False

    for obs in obslist:
        #print "obs here"
        if obs.type == 'moving':

            xcoord = math.cos(robotPose.theta)*obs.x - math.sin(robotPose.theta)*obs.y + robotPose.x
            ycoord = -math.sin(robotPose.theta)*obs.x + math.cos(robotPose.theta)*obs.y + robotPose.y
            if((xcoord > 0 and xcoord < course.height) and (ycoord < course.width/2 - .5 and ycoord > -course.width/2 + .5)):
                #print 'moving'
                if (stopMode == False and ignoreObs < 3):
                    obsTime = int(round(time.time() * 1000))
                    stop()
                    ignoreObs = ignoreObs + 1

    '''if(not firstPose):
        return
    for i in range(0,len(obslist)):
        isFound = False

        xcoord = math.cos(robotPose.theta)*obslist[i].x - math.sin(robotPose.theta)*obslist[i].y + robotPose.x
        ycoord = -math.sin(robotPose.theta)*obslist[i].x + math.cos(robotPose.theta)*obslist[i].y + robotPose.y
        #print ("(" + str(xcoord) + ', ' + str(ycoord) + ')')

        if((xcoord > 0 and xcoord < course.height) and (ycoord < course.width/2 and ycoord > -course.width/2)):
            #print("obs in world frame " + str(course.height) + 'x' + str(course.width))
            print("Pose:" + str(robotPose.theta) + " rx:" + str(robotPose.x) + " ry:" + str(robotPose.y) + " ox:" + str(obslist[i].x) + " oy:" + str(obslist[i].y))

            for curobs in obstacles:
                dist = distance(xcoord, ycoord , curobs.x, curobs.y)
                print(" x:" + str(xcoord) + " y:" + str(ycoord) + " x2:" + str(curobs.x) + " y2:" + str(curobs.y) + " dist:" + str(dist))
                #print(dist)
                print("Obstacles" + str(len(obstacles)))
                if(dist < .8 and obslist[i].type == 'static'):
                    if(not isFound):
                        isFound = True
                        print("obs already detected")
                        curobs.x = obslist[i].x
                        curobs.y = obslist[i].y
                        #break
                    else:
                        print("duplicate found")
                        obstacles.remove(curobs)
            #if(isFound):
                #break
            if(not isFound and obslist[i].type == 'static'):
                print("new obstacle added!")
                print(obslist[i].type)
                print ("(" + str(xcoord) + ', ' + str(ycoord) + ')')
                obstacles.append(obstacle(xcoord,ycoord))
'''
def listener():
    rospy.Subscriber("zenith/pose2D", Pose2D, poseCallback,queue_size=1)
    #rospy.Subscriber("turtle1/pose", Pose, poseCallback,queue_size=1)
    rospy.Subscriber("zenith/obstacles",ObstacleList,obscallback,queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
