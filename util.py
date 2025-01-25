import math
import re

def ema(current, previous, smooth):
    return (current * smooth) + (previous * (1 - smooth))

def sanitize_angle(angle, degrees=False):
    if degrees == False:
        return (angle % (2 * math.pi) + 2 * math.pi) % (2 * math.pi)
    else:
        return (angle % 360 + 360) % 360

def stringToCoords(coordinates, placeholder = True):
    val = re.sub(r',', ' ', coordinates)
    li = val.split(" ")
    temp = []
    for index in range(len(li)):
        i = li[index]
        if index == 3:
            continue
        if i == "":
            pass
        else:
            temp.append(float(i))

    if (placeholder == True):
        temp.insert(2, -999) # theta placeholder, push velocity back

    return temp

def findFileIndex(pathFile, lookup):
    for num, line in enumerate(pathFile, 1):
        if lookup in line:
            return num

def calculateLinVelocity(motorRPM, gearRatio, wheelDiameter):    
    return 2 * math.pi * (wheelDiameter / 2) * (motorRPM * gearRatio / 60)

def split_numbers(input_string):
    numbers = input_string.split(", ")
    
    grouped_numbers = []
    
    for i in range(0, len(numbers), 2):
        grouped_string = f"{numbers[i]}, {numbers[i+1]}"
        grouped_numbers.append(grouped_string)
    
    return grouped_numbers

#co-pilot again (really lazy)
def calculateAngle(p1, p2, deg):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle = math.atan2(dx, dy)
    if (deg == True):
        return sanitize_angle(math.deg(angle), deg) # Convert radians to degrees if needed and normalize it to [0, 360]
    return sanitize_angle(angle, deg)

def sgn(num):
    if (num >= 0):
        return 1
    else:
        return -1
    
def calculateCurvature(current, next, theta, inDegrees):
    # find which side the point is on the tracking circle

    if (inDegrees == True):
        theta = math.radians(theta)

    dist = math.hypot(next[0] - current[0], next[1] - current[1])

    if (dist == 0):
        return -999

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