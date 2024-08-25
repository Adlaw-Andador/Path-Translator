from pprint import pprint
import math
import numpy as np
import re
import os

#file name to translate
fileName = "uwuwuwuwuw.txt"
#file name to export as
exportName = "testTry.txt"

pathToPathFile = os.path.dirname(__file__) + "\\paths\\" + fileName
pathToExports = os.path.dirname(__file__) + "\\exports\\" + exportName

pathFile = open(pathToPathFile)
path = []
controlPoints = []

#co-pilot cuz im lazy
def split_numbers(input_string):
    numbers = input_string.split(", ")
    
    grouped_numbers = []
    
    for i in range(0, len(numbers), 2):
        grouped_string = f"{numbers[i]}, {numbers[i+1]}"
        grouped_numbers.append(grouped_string)
    
    return grouped_numbers

#co-pilot again (really lazy)
def calculateAngle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle = math.atan2(dx, dy)
    #return math.degrees(angle)  # Convert radians to degrees if needed
    return angle

def sgn(num):
    if (num >= 0):
        return 1
    else:
        return -1

def calculateCurvature(current, next, theta):
    # find which side the point is on the tracking circle

    #dist = math.hypot(next[0] - current[0], next[1] - current[1])

    #if (dist == 0):
    #    return -999

    if (current[3] == 0): return -999
    
    if theta == 0.0:
        side = sgn(next[1] - current[1])
        x = math.fabs(next[1] - current[1])
        d = math.hypot(next[0] - current[0], next[1] - current[1])
    else:
        side = sgn(math.sin(theta) * (next[0] - current[0]) - math.cos(theta) * (next[1] - current[1]))

        a = -math.tan(theta)
        c = math.tan(theta) * current[0] - current[1]
        x = math.fabs(a * next[0] + next[1] + c) / math.sqrt(math.pow(a, 2) + 1)
        d = math.hypot(next[0] - current[0], next[1] - current[1])

    return side * ((2 * x) / math.pow(d, 2))

def findFileIndex(lookup):
    for num, line in enumerate(pathFile, 1):
        if lookup in line:
            return num

def stringToCoords(coordinates, placeholder = True):
    val = re.sub(r',', ' ', coordinates)
    li = val.split(" ")
    coordinates = []
    for i in li:
        if i == "":
            pass
        else:
            coordinates.append(float(i))

    if (placeholder == True):
        coordinates.insert(2, -999) # theta placeholder, push velocity back

    return coordinates

def translate():

    for _, line in enumerate(pathFile, 1):
        striped = line.strip('\n')
        if (striped == "endData"):
            break
        path.append(stringToCoords(striped))

    for _, line in enumerate(pathFile, 1):
        striped = line.strip('\n')
        if (striped.find("#PATH.JERRYIO-DATA") != -1): break
        if (striped.find(",") != -1):
            for ctrlPnt in split_numbers(striped):
                controlPoints.append(stringToCoords(ctrlPnt, False))
    
def translateToRamsete():
    # calculate angle and change velocities to 12volt max
    for index in range(len(path) - 1):
        angle = calculateAngle(path[index], path[index + 1])
        path[index][2] = angle

        if (index == len(path) - 2): 
            path[index][2] = 0.0
            path[index + 1][2] = 0.0
        
        path[index][3] *= (12/127)

    for index in range(len(path) - 1):
        curvature = calculateCurvature(path[index], path[index + 1], path[index][2])
        if (curvature == -999):
            path[index].append(0)
        
        path[index].append(path[index][3] * curvature)
    
    pprint(path)

def exportPath():
    with open(pathToExports, 'w') as file:
        for row in path:
            line = ', '.join(map(str, row))

            file.write(line + '\n')

translate()
translateToRamsete()
exportPath()
