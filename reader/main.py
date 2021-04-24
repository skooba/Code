import armCOMMAND
import cannyEdge
import featureMatching
import readData
import inverseKinematics
import matplotlib.pyplot as plt
import serial
import math

displayWidth = 160
displayHeight = 120

port = "COM3"
toRadians = 2 * math.pi / 360
minAngle1 = toRadians * 85
minAngle2 = toRadians * 35
minAngle3 = toRadians * 135
maxAngle1 = toRadians * 290
maxAngle2 = toRadians * 319
maxAngle3 = toRadians * 225

ser = serial.Serial(port, 5000000)

saveImg = 0  # saves the first image taken by the camera as the canny image to work off of
cannyAdjust = 0  # setting this to 1 send to the process of adjusting Canny threshold values

while True:
    try:  # If not samples are found then finds error and throws back to top
        arrImage, dataList = readData.readData(ser, displayWidth, displayHeight)
        cannyImage = cannyEdge.getEdges(arrImage, cannyAdjust)
        xCentroid, yCentroid = featureMatching.orbMatch(cannyImage, displayHeight, saveImg)

        plt.clf()
        plt.imshow(arrImage, cmap='gray', vmin=0, vmax=255)
        xEffector = displayWidth / 2
        yEffector = displayHeight / 2
        plt.scatter(xEffector, yEffector, marker='D')
        plt.scatter(xCentroid, displayHeight - yCentroid, marker='x', color='red')
        plt.arrow(xEffector, yEffector, .3 * (xCentroid - xEffector), .3 * (displayHeight - yCentroid - yEffector),
                  width=.5, color='green')
        plt.show(block = False)
        plt.pause(.01)
        #

        angleList, radiansPerCount1, radiansPerCount2, radiansPerCount3 = armCOMMAND.getAngles(dataList)

        jacobianMatrix, xScale, yScale = inverseKinematics.jacobian(3.54, 4.72, 4.125, angleList[0], angleList[1],
                                                                    angleList[2])

        xCartesian, yCartesian, = inverseKinematics.cartVelocities(xCentroid, yCentroid, xEffector, yEffector)
        xScaled = xCartesian * .5
        yScaled = yCartesian * .5

        print("angle  1 = ", angleList[0], "   angle  2 = ", angleList[1], "   angle  3 = ", angleList[2])

        qDot = inverseKinematics.solveQP(0, minAngle1, minAngle2, minAngle3, maxAngle1, maxAngle2, maxAngle3,
                                         jacobianMatrix, xScaled, yScaled,
                                         angleList[0], angleList[1], angleList[2])

        armCOMMAND.driveMotors(ser, qDot[0], qDot[1], qDot[2], radiansPerCount1, radiansPerCount2, radiansPerCount3)
        continue
    except ZeroDivisionError:
        print("This is a DIVIDED BY ZERO error")
        continue
