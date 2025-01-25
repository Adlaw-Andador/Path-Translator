import os
import math
from pprint import pprint
import util

pathsFolder = os.path.dirname(__file__) + "\\paths\\"
exportFolder = os.path.dirname(__file__) + "\\exports\\"

class PathTranslator:

    __path = []

    __targetFileName = "None"

    __odometrySpeeds = [[0.0, 0.0, 0.0]]

    __odometrySpeedsExport = False
    __timeStamps = False
    
    def __writeInFile(self, file):
        for row in self.__path:
            line = ', '.join(f"{num:.3f}" for num in row)
            file.write(line + ',' + '\n')

    def __exportPath(self, timeIncluded, odometrySpeeds = False):
        with open(exportFolder + self.__exportName, 'w') as file:
            if (timeIncluded == True and odometrySpeeds == True):
                file.write('time, x, y, theta, xVelocity, yVelocity, thetaVelocity' + '\n')
                self.__writeInFile(file)
            elif (timeIncluded == True and odometrySpeeds == False):
                file.write('time, x, y, theta, lin_vel, ang_vel' + '\n')
                self.__writeInFile(file)
            elif (timeIncluded == False and odometrySpeeds == True):
                file.write('time, x, y, theta, xVelocity, yVelocity, thetaVelocity' + '\n')
                self.__writeInFile(file)
            elif (timeIncluded == False and odometrySpeeds == False):
                file.write('x, y, theta, lin_vel, ang_vel' + '\n')
                self.__writeInFile(file)                                    

    def __txtPathToArray(self):

        pathFile = open(pathsFolder + self.__targetFileName)

        for _, line in enumerate(pathFile, 1):
            striped = line.strip('\n')
            if (striped == "endData" or striped.find("#PATH.JERRYIO-DATA") != -1 or striped == "#PATH-POINTS-START Path"):
                continue
            self.__path.append(util.stringToCoords(striped))

        
        for index in range(len(self.__path) - 1):
            coordinates = self.__path[index]

            coordinates[0] /= 2.54
            coordinates[1] /= 2.54


        # for _, line in enumerate(pathFile, 1):
        #     striped = line.strip('\n')
        #     if (striped.find("#PATH.JERRYIO-DATA") != -1): break
        #     if (striped.find(",") != -1):
        #         for ctrlPnt in util.split_numbers(striped):
        #             self.__controlPoints.append(util.stringToCoords(ctrlPnt, False))
        
        #pprint(self.__path)

    def __translateToRamsete(self):
        for index in range(len(self.__path) - 1):
            angle = util.calculateAngle(self.__path[index], self.__path[index + 1], self.__exportUnits[0])
            self.__path[index][2] = angle

            if (index == len(self.__path) - 2): 
                self.__path[index + 1][2] = 0.0
            
            if (self.__exportUnits[1] == "linearVelocity"): self.__path[index][3] = util.calculateLinVelocity(self.__path[index][3], self.__robotParameters[1], self.__robotParameters[0])

        for index in range(len(self.__path) - 1):
            curvature = util.calculateCurvature(self.__path[index], self.__path[index + 1], self.__path[index][2], self.__exportUnits[0])
            if (curvature == -999):
                self.__path[index].append(0)
            
            self.__path[index].append(self.__path[index][3] * curvature)

    def __calculatePathTime(self):
        pointsFormatted = []
        totalTime = 0

        for i in range(len(self.__path) - 1):
            if (len(self.__path[i]) != 5 or len(self.__path[i + 1]) != 5):
                pointsFormatted.insert(i, [totalTime, self.__path[i][0], self.__path[i][1], self.__path[i][2], 0, 0])
                print("sampha time: ", totalTime)
            else:
                x1, y1, theta1, linearVelocity1, angularVelocity1 = self.__path[i]
                x2, y2, theta2, linearVelocity2, angularVelocity2 = self.__path[i + 1]

                dx = x2 - x1
                dy = y2 - y1
                distance = math.sqrt(dx**2 + dy**2)

                if (self.__exportUnits[0] == True):
                    dTheta = util.sanitize_angle(math.radians(theta2 - theta1), False)
                else:
                    dTheta = util.sanitize_angle(theta2 - theta1, False)

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

                totalTime += timeAtPoint

                pointsFormatted.insert(i, [totalTime - timeAtPoint, x1, y1, theta1, linearVelocity1, angularVelocity1])
        
        print("Path time: ", totalTime)
        self.__path = pointsFormatted
        #pprint("Path: ", pointsFormatted)
    
    def __replaceVelocities(self):
        for index in range(len(self.__path)):
            xVelocity, yVelocity, thetaVelocity = self.__odometrySpeeds[index]

            self.__path[index][4] = xVelocity
            self.__path[index][5] = yVelocity
            self.__path[index].append(thetaVelocity)
    
    def __calculateSpeeds(self):
        previousTime, previousX, previousY, previousTheta, _, _ = self.__path[0]
        previousSpeed = [0.0, 0.0, 0.0]
        
        for index in range(1, (len(self.__path))):
            currentTime, currentX, currentY, currentTheta, _, _ = self.__path[index]

            deltaTime = currentTime - previousTime

            currentSpeeds = [
                (currentX - previousX) / deltaTime,
                (currentY - previousY) / deltaTime,
                (currentTheta - previousTheta) / deltaTime
                # util.ema((currentX - previousX) / deltaTime, previousSpeeds[0], 0.95),
                # util.ema((currentY - previousY) / deltaTime, previousSpeeds[1], 0.95),
                # util.ema((currentTheta - previousTheta) / deltaTime, previousSpeeds[2], 0.95)
            ]

            self.__odometrySpeeds.append(currentSpeeds)

            previousSpeeds = currentSpeeds
            previousTime = currentTime
            previousX = currentX
            previousY = currentY
            previousTheta = currentTheta


    def __init__(self, exportName, exportUnits, robotParameters):
        """Create a new translator object to take jerry io files and translate to ramsete

        Args:
            exportName (string): The name to call the .txt file (no .txt extension required)
            exportUnits (list): [inDegrees (true/false), speedUnits ("linearVelocity"/"default")]
            robotParameters (list): [wheelDiameter, gearRatio]
        
        """

        self.__exportName = exportName + ".txt"
        self.__exportUnits = exportUnits
        self.__robotParameters = robotParameters
    
    def selectFile(self, fileName):
        self.__targetFileName = fileName + ".txt"

    def translatePath(self, addTimeStamps, odomSpeeds):
        self.__txtPathToArray()
        self.__translateToRamsete() 
        # for index in range(len(self.__path) - 1):
        #     print(len(self.__path[index]))
        if (addTimeStamps == True): self.__calculatePathTime()
        if (odomSpeeds == True): 
            self.__calculateSpeeds()
            self.__replaceVelocities()
        
        self.__timeStamps = addTimeStamps
        self.__odometrySpeedsExport = odomSpeeds
        
    def getPath(self):
        return self.__path
    
    def getOdomSpeeds(self):
        return self.__odometrySpeeds


    def downloadFile(self):
        self.__exportPath(self.__timeStamps, self.__odometrySpeedsExport)
        print("Exported to: ", pathsFolder + "\\" + self.__exportName)
        print("Data Length: ", len(self.__path))



