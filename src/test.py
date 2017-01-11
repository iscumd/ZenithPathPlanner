import math
x = input("CurrX?")
y = input("CurrY?")
theta = input("CurrTheta?")

newx = input("NewX?")
newy = input("NewY?")
print(" \n ")
print("------------------------")
print("Going from (" + str(x) + ", " + str(y) + ") to (" + str(newx) + ", " + str(newy) + ").")
print("\n -------------------------")
def distance(x1,y1,x2,y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def angle_diff(a0, a1):
    return (((a0 - a1) + 3 * math.pi) % (2 * math.pi)) - math.pi

d = distance(x, y,newx,newy)

heading_to_p = math.atan2(newy - y, newx - x)
print("Heading to Point: " + str(heading_to_p))
heading_error = angle_diff(theta, heading_to_p)
print("Error: " + str(heading_error))
