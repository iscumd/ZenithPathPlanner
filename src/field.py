import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class point(object):
    def __init__(self,x,y):
        self.x = x
        self.y = y
    def __str__(self):
        return ("(" + str(self.x) + ', ' + str(self.y) + ')')

class obstacle(object):
    def __init__(self,c,k,x,y):
        self.c = c
        self.k = k
        self.x = x
        self.y = y
class field(object):
    fig = plt.figure()
    #ax = fig.add_subplot(111,projection='3d')
    goals = []
    pathToCurrGoal = []
    currGoal = []
    robotPos = point(0,0)
    obstacles = []
    height = 15
    width = 4
    def __init__(self,pos,goal):
        #print "fieldInit"
        self.robotPos = pos
        self.goals.append(goal)
    def distance(self,x1,y1,x2,y2):
        return ((x2-x1)**2 + (y2-y1)**2)**(0.5)
    def magnitude(self,x,y,z):
        return (x**2 + y**2 + z**2)**(.5)
    def gradient(self,k,c,a,b,x,y): #k c are tuners a b are obstacle loc x y are robotpos
        v = [0,0]
        if (((a-x)**2+(b-y)**2) != 0):
            kc = k*c*((a-x)**2+(b-y)**2)**((-2-c)/2)
            v[0] = kc*(a-x)
            v[1] = kc*(b-y)
        else:
            v = [0,0]
        return v
    def cost(self,x,y):
        #print(self.goals[0].x)
        c = self.singleCost(x,y,1.1,-10,self.goals[0].x,self.goals[0].y)
        #c=0
        for obstacle in self.obstacles:
            c = c + self.singleCost(x,y,obstacle.c,obstacle.k,obstacle.x,obstacle.y)
        return c
    def boundGradient(self):
        g = []
        d1 = (self.height-self.robotPos.x)
        if(d1):
            g.append(d1**-2)
        else:
            g.append(0)
        d2 = self.robotPos.x
        if(d2):
            g.append(-(d2**-2))
        else:
            g.append(0)
        d3 = ((self.width/2)-self.robotPos.y)
        if(d3):
            g.append(d3**-2)
        else:
            g.append(0)
        d4 = (self.robotPos.y + (self.width/2))
        if(d4):
            g.append(-(d4**-2))
        else:
            g.append(0)
        return g
    def findGradientVector(self):
        gv = [0,0]
        vgoal = [0,0]
        bg = self.boundGradient()
        #self.height = 15
        #self.width = 4
        #print(self.robotPos.x)
        vgoal = self.gradient(-100,1.5,self.goals[0].x,self.goals[0].y,self.robotPos.x,self.robotPos.y)
        gv[0] = vgoal[0]
        gv[1] = vgoal[1]
        gv[0] = gv[0] + bg[0] + bg[1]
        gv[1] = gv[1] + bg[2] + bg[3]
        for obstacle in self.obstacles:
            v = self.gradient(obstacle.k,obstacle.c,obstacle.x,obstacle.y,self.robotPos.x,self.robotPos.y)
            gv[0] = gv[0] + v[0]
            gv[1] = gv[1] + v[1]
        return gv
    def singleCost(self,x,y,c,k,a,b):
        zinf = 10
        if(self.distance(a,b,x,y)**c == 0):
            if(k<0):
                s = -1
                return zinf*s
            else:
                return zinf
        else:
            return k/(self.distance(a,b,x,y)**c)
    def newPotentialFieldPath(self,xstart,ystart):
        isFinished = 0
        j = 0
        self.robotPos.x = xstart
        self.robotPos.y = ystart
        self.pathToCurrGoal.append([self.robotPos.x])
        self.pathToCurrGoal.append([self.robotPos.y])

        while(not isFinished and j<100):
            distpathtogoal = self.distance(self.robotPos.x,self.robotPos.y,self.goals[0].x,self.goals[0].y)
            #distpathtogoal = self.distance(self.pathToCurrGoal[len(self.pathToCurrGoal)-1][0],self.pathToCurrGoal[len(self.pathToCurrGoal)-1][1], self.goals[0].x,self.goals[0].y)
            #print(distpathtogoal)
            if( distpathtogoal > .2):
                #print "(" + str(self.robotPos.x) + ", " + str(self.robotPos.y) + ')'
                gv = self.findGradientVector()
                mg = self.magnitude(gv[0],gv[1],0)
                #self.pathToCurrGoal.append([self.robotPos.x + (gv[0]/mg)/5,self.robotPos.y + (gv[1]/mg)/5])
                self.pathToCurrGoal[0].append(self.robotPos.x + -(gv[0]/mg)*.5)
                self.pathToCurrGoal[1].append(self.robotPos.y + -(gv[1]/mg)*.5)
                #print((gv[0]/mg))
                self.robotPos.x = self.robotPos.x + -(gv[0]/mg)*.5
                self.robotPos.y = self.robotPos.y + -(gv[1]/mg)*.5

                j = j+1
            else:
                isFinished = 1
        #print j

        #plt.plot(self.pathToCurrGoal[0],self.pathToCurrGoal[1])
        #plt.show()
    def newPerpendicularPath(self,pose):
        '''
        sample path headed to waypoint for heightest cost return xh,yh
        sample path perpendicular to heading to waypoint and intersecting with xh,yh for lowest cost
        '''
        stepSize = 0.1
        localMax = point(0,0)
        #print(int(self.distance(pose.x,pose.y,self.goals[0].x,self.goals[0].y)))
        for i in range(0,int(self.distance(pose.x,pose.y,self.goals[0].x,self.goals[0].y)*1/stepSize)):
            i = i*stepSize
            sample = [self.cost(pose.x + (i-stepSize)*math.cos(pose.theta),pose.y + (i-stepSize)*math.sin(pose.theta)),self.cost(pose.x + i*math.cos(pose.theta),pose.y + i*math.sin(pose.theta)),self.cost(pose.x + (i+stepSize)*math.cos(pose.theta),pose.y + (i+stepSize)*math.sin(pose.theta))]
            if(sample[2] < sample[1] and sample[0] < sample[1]): #local maximum detected
                localMax.x = pose.x + i*math.cos(pose.theta)
                localMax.y = pose.y + i*math.sin(pose.theta)
                break;
        loopi = 0
        lowest = 1000
        localMin = point(0,0)
        rinbound = True
        linbound = True
        while(linbound or rinbound): # while either left bound or right bound is still inside the field keep sampling points
            loopi = loopi + stepSize
            sample = point(localMax.x + (loopi)*math.cos(pose.theta+math.pi/2),localMax.y + (loopi)*math.sin(pose.theta+math.pi/2))
            negSample = point(localMax.x + (-loopi)*math.cos(pose.theta),localMax.y + (-loopi)*math.sin(pose.theta))
            if(rinbound and sample.x > 0 and sample.x < self.height and sample.y > -self.width/2 and sample.y < self.width/2):
                rinbound = True
            else:
                rinbound = False
            if((self.cost(sample.x,sample.y) < lowest) and rinbound):
                    lowest = self.cost(sample.x,sample.y)
                    localMin.x = sample.x
                    localMin.y = sample.y
            if(linbound and negSample.x > 0 and negSample.x < self.height and negSample.y > -self.width/2 and negSample.y < self.width/2):
                linobound = True
            else:
                linbound = False
            if(linbound and self.cost(negSample.x,negSample.y) < lowest):
                    lowest = self.cost(negSample.x,negSample.y)
                    localMin.x = negSample.x
                    localMin.y = negSample.y
        self.goals.insert(0,localMin)
        print("new point at (" + str(localMin.x) + ', ' + str(localMin.y) + ')')


if(__name__ == "__main__"):
    main()
