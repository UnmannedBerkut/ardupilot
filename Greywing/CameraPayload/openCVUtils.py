import numpy
import transforms3d
import math

def lim(input, high, low):
    if (input > high):
        return high
    if(input < low):
        return low
    return input

def rotateEarth2Camera(point,roll,pitch,yaw):
    #point: x,y,z: where x in into the screen, y is to the right, and z is down, 0,0,0 is screen center
    #calculate the direction cosine matrix for the supplied attitude
    R = transforms3d.euler.euler2mat(math.radians(roll), math.radians(pitch), math.radians(yaw), 'sxyz')

    return numpy.matmul(numpy.transpose(R), point)

def rotateCamera2Earth(point,roll,pitch,yaw):
    #calculate the direction cosine matrix for the supplied attitude
    R = transforms3d.euler.euler2mat(math.radians(roll), math.radians(pitch), math.radians(yaw), 'sxyz')

    return numpy.matmul(R, point)

#Convert from (0,0) center to (0,0) upper right
def convertCamera2OpenCV(point):
    #x,y in OpenCV camera coordinates (0,0 is upper left)
    #assumes 640x480
    return [ point[1][0]+320, point[2][0]+240 ]

