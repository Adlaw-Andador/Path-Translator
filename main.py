from pprint import pprint
import math
import numpy as np
import re
import os
import matplotlib.pyplot as plt
import util
from translator import PathTranslator

#file name to translate
fileName = "kendricklamar.txt"
#file name to export as
exportName = "kendricklamarTest.txt"
# export as degrees/radians
exportThetaAsDeg = False
# export as volts or inches per second, radians per second
exportVoltageUnits = False
MAX_VOLTAGES = 12
MAX_LINEAR_SPEED = 456

#dont touch
exportFormat = True

pathToPathFile = os.path.dirname(__file__) + "\\paths\\" + fileName
pathToExports = os.path.dirname(__file__) + "\\exports\\" + exportName

pathFile = open(pathToPathFile)
path = []
pointsFormatted = []
controlPoints = []
    
temporaryDTs = []
    
def calculatePathTime(points):
    totalTime = 0

    for i in range(len(points) - 1):


        if (len(points[i]) > 5 or len(points[i + 1]) > 5):
            pointsFormatted.insert(i, [totalTime, points[i][0], points[i][1], points[i][2], 0, 0])
            print("sampha time: ", totalTime)
        else:
            x1, y1, theta1, linearVelocity1, angularVelocity1 = points[i]
            x2, y2, theta2, linearVelocity2, angularVelocity2 = points[i + 1]

            dx = x2 - x1
            dy = y2 - y1
            distance = math.sqrt(dx**2 + dy**2)

            dTheta = (theta2 - theta1) % math.radians(360) # normalize angle

            # linear interpolation, maybe find another method
            averageLinearVelocity = (linearVelocity1 + linearVelocity2) / 2
            averageAngularVelocity = (angularVelocity1 + angularVelocity2) / 2

            #print(f"Segment {i} - Distance: {distance}, LinearVelocities: {linearVelocity1}, {linearVelocity2}, Delta Theta: {dTheta}, AngularVelocities: {angularVelocity1}, {angularVelocity2}")
        # print(f"Segment {i} - dTheta: {dTheta}, AngularVelocities: {angularVelocity1}, {angularVelocity2}")

            tLinear = distance / averageLinearVelocity
            tAngular = dTheta / averageAngularVelocity

            print(f"Segment {i} - dTheta: {dTheta}, avgAngular: {averageAngularVelocity}")
            print(f"Segment {i} - tLinear: {tLinear}, tAngular: {tAngular}")

            timeAtPoint = max(tLinear, tAngular) # to obey the laws of physics, the one that takes more time will be prioritized

            print(timeAtPoint)
            temporaryDTs.append(timeAtPoint)

            totalTime += timeAtPoint

            pointsFormatted.insert(i, [totalTime - timeAtPoint, x1, y1, theta1, linearVelocity1, angularVelocity1])
    
    print("Path time: ", totalTime)
    #pprint("Path: ", pointsFormatted)

def translate():
    for _, line in enumerate(pathFile, 1):
        striped = line.strip('\n')
        if (striped == "endData" or striped.find("#PATH.JERRYIO-DATA") != -1):
            break
        path.append(util.stringToCoords(striped))

    for _, line in enumerate(pathFile, 1):
        striped = line.strip('\n')
        if (striped.find("#PATH.JERRYIO-DATA") != -1): break
        if (striped.find(",") != -1):
            for ctrlPnt in util.split_numbers(striped):
                controlPoints.append(util.stringToCoords(ctrlPnt, False))

temporaryLinears = []
temporaryAngulars = []    

def translateToRamsete():
    # calculate angle and change velocities to 12volt max
    for index in range(len(path) - 1):
        angle = util.calculateAngle(path[index], path[index + 1], exportThetaAsDeg)
        path[index][2] = angle

        if (index == len(path) - 2): 
            path[index][2] = 0.0
            path[index + 1][2] = 0.0
        
        temporaryLinears.insert(index, util.calculateLinVelocity(path[index][3], 36/48, 3.25))

        if (exportVoltageUnits == False): path[index][3] = util.calculateLinVelocity(path[index][3], 36/48, 3.25)

    for index in range(len(path) - 1):
        curvature = util.calculateCurvature(path[index], path[index + 1], path[index][2])
        if (curvature == -999):
            path[index].append(0)
        
        temporaryAngulars.insert(index, path[index][3] * curvature)
        path[index].append(path[index][3] * curvature)
    
    pprint(path)

def exportPath():
    # im lazy sorry
    with open(pathToExports, 'w') as file:
        if (exportFormat == True):
            file.write('time, x, y, theta, lin_vel, ang_vel' + '\n')
            for row in pointsFormatted:
                line = ', '.join(f"{num:.3f}" for num in row)
                file.write(line + ',' + '\n')
        else:
            file.write('x, y, theta, lin_vel, ang_vel' + '\n')
            for row in path:
                line = ', '.join(f"{num:.3f}" for num in row)
                file.write(line + '\n')

main = PathTranslator(\
    "imaTestFile", 
    [
        False,
        "linearVelocity"
    ], 
    [
        3.25,
        36/48
    ]
)
main.selectFile("kendricklamar2")
main.translatePath(True, True)
main.downloadFile()

currentPath = main.getPath()
odomSpeeds = main.getOdomSpeeds()

# translate()
# translateToRamsete()
# calculatePathTime(path)
# exportPath()

def simulateOdometry(initialPose):

    currentX, currentY, currentTheta = initialPose
    odometryData = [(currentX, currentY, currentTheta)]

    previousTime = 0.0

    for i in range (len(odomSpeeds)):
        currentTime = currentPath[i][0]
        xVelocity, yVelocity, thetaVelocity = odomSpeeds[i]
        deltaTime = currentTime - previousTime

        # print(f"xVel: {}, yVel: {}, ")

        currentX += xVelocity * deltaTime
        currentY += yVelocity * deltaTime
        currentTheta += thetaVelocity * deltaTime

        odometryData.append((currentX, currentY, currentTheta))

        previousTime = currentTime
    
    return odometryData


# def simulate_odometry(initial_pose):
#     """
#     Simulates odometry coordinates for a differential drive robot.
    
#     Args:
#         linear_velocities (list of float): Linear velocities (m/s).
#         angular_velocities (list of float): Angular velocities (rad/s).
#         time_intervals (list of float): Time intervals between measurements (s).
#         initial_pose (tuple): Initial pose of the robot as (x, y, theta) in meters and radians.
        
#     Returns:
#         list of tuple: A list of (time, x, y, theta) tuples representing the robot's pose over time.
#     """
#     # Initialize variables
#     x, y, theta = initial_pose
#     odometry_data = [(0.0, x, y, theta)]  # Start with the initial pose

#     previousTime = 0.0
    
#     # Simulate odometry updates
#     for i in range(len(currentPath)):

#         currentTime, _, _, _, linearVelocity, angularVelocity = currentPath[i]
#         deltaTime = (currentTime - previousTime)

#         if (angularVelocity == 0):
#             radius = 0
#         else:
#             radius = linearVelocity / angularVelocity

#             deltaTheta = angularVelocity * deltaTime

#             nextTheta = theta + deltaTheta

#             centerX = x - radius * math.sin(theta) 
#             centerY = y + radius * math.cos(theta)

#             # nextX = centerX + radius * math.cos(nextTheta)
#             # nextY = centerY + radius * math.sin(nextTheta)

#             previousTime = currentTime

#             x = centerX + radius * math.cos(deltaTheta)
#             y = centerY + radius * math.sin(deltaTheta)
#             theta += deltaTheta
    
#         # Append new pose to odometry data
#         odometry_data.append((currentTime, x, y, theta))
    
#     return odometry_data


# # print("Exported to: ", pathToExports + "\\" + exportName)
initial_pose = (63.233, 0.362, 4.188)

# Simulate odometry
odometry = simulateOdometry(initial_pose)

# Print results
# Extract data for plotting
x_coords = [pose[0] for pose in odometry]
y_coords = [pose[1] for pose in odometry]

#Plot the trajectory
plt.figure(figsize=(8, 6))
plt.plot(x_coords, y_coords, marker='o', label="Robot Path")
plt.title("Robot Trajectory Using Odometry")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.grid(True)
plt.legend()
plt.axis([-72, 72, -72, 72])
#plt.axis('equal')  # Keep the aspect ratio equal for better visualization
plt.show()