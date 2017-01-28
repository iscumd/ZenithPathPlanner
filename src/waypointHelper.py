import math
from geometry_msgs.msg import Twist

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

def getTwist(pose,newx,newy,dir):
    linearTune = 10.0
    angularTune = 1

    d = distance(pose.x, pose.y,newx,newy)

    heading_to_p = math.atan2(newy - pose.y, newx - pose.x)
    #print("Heading to Point: " + str(heading_to_p))
    if(dir == -1):
        heading_error = angle_diff( heading_to_p,angle_diff(pose.theta, math.pi))
        print("Reverse Mode")
    else:
        heading_error = angle_diff(heading_to_p, pose.theta)
    #if(abs(heading_error) < .08):
    #    heading_error = 0
    #print("Error: " + str(heading_error))
    w = chop(angularTune * heading_error, -0.5, 0.5)

    v = chop(1.0 / (linearTune * abs(heading_error) + .000000001), 0.0, .5)

    if(dir == -1):
        v = v*-1

    dpose = Twist()
    dpose.linear.x = v
    dpose.angular.z = w
    #print(str(dpose))

    return dpose


class point(object):
    def __init__(self,x,y):
        self.x = x
        self.y = y

class obstacle(object):
    def __init__(self,x,y):
        self.x = x
        self.y = y
class box(object):
    def __init__(self,height,width,x,y):
        self.height = height
        self.width = width
        self.x = x
        self.y = y
