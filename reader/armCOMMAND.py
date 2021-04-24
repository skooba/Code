import math
import time
import struct


def driveMotors(ser, q1dot, q2dot, q3dot, radiansPerCount1, radiansPerCount2, radiansPerCount3):
    ser.write(bytearray([0x19, 0X19]))  # tells Arduino to begin arm moving sequence

    countsQ1 = q1dot / radiansPerCount1
    countsQ2 = q2dot / radiansPerCount2
    countsQ3 = q3dot / radiansPerCount3

    posQ1 = round(abs(countsQ1))
    posQ2 = round(abs(countsQ2))
    posQ3 = round(abs(countsQ3))
    print('counts Q1 = ', countsQ1, ' counts Q2 = ', countsQ2, 'counts Q3 = ', countsQ3)

    if posQ1 + posQ2 + posQ3 > 60:  # if we have a singular KKT matrix from the optiization problem the values will be too high so we scale them down
        modQ1 = round(60 * posQ1 / (abs(countsQ1) + abs(countsQ2) + int(abs(countsQ3))))
        modQ2 = round(60 * posQ2 / (abs(countsQ1) + abs(countsQ2) + int(abs(countsQ3))))
        modQ3 = round(60 * posQ3 / (abs(countsQ1) + abs(countsQ2) + int(abs(countsQ3))))
    else:
        modQ1 = posQ1
        modQ2 = posQ2
        modQ3 = posQ3

    # since we are giving ints (0-255), we also have to send the sign of the data
    signQ1 = 0
    signQ2 = 0
    signQ3 = 0
    if countsQ1 >= 0:
        signQ1 = 1
    if countsQ2 >= 0:
        signQ2 = 1
    if countsQ3 >= 0:
        signQ3 = 1

    time.sleep(.5)
    ser.write(struct.pack('>BBBBBBB', modQ1, signQ1, modQ2, signQ2, modQ3, signQ3,
                          255))  # 255 IS THE STOP BIT

    print('mod Q1 = ', modQ1, ' mod Q2 = ', modQ2, 'mod Q3 = ', modQ3)


def getAngles(data_list):
    toRadians = 2 * math.pi / 360
    minCount1 = 155
    minCount2 = 770
    minCount3 = 133
    maxCount1 = 688
    maxCount2 = 243
    maxCount3 = 450
    min1 = toRadians * 90
    min2 = toRadians * 90
    min3 = toRadians * 126
    max1 = toRadians * 270
    max2 = toRadians * 270
    max3 = toRadians * 233
    radiansPerCount1 = (max1 - min1) / (maxCount1 - minCount1)
    radiansPerCount2 = (max2 - min2) / (minCount2 - maxCount2)
    radiansPerCount3 = (max3 - min3) / (maxCount3 - minCount3)
    counts1 = data_list[0]
    counts2 = data_list[1]
    counts3 = data_list[2]
    angleList = [0] * 3
    angleList[0] = min1 + radiansPerCount1 * (counts1 - minCount1)
    angleList[1] = max2 - radiansPerCount2 * (counts2 - maxCount2)
    angleList[2] = min3 + radiansPerCount3 * (counts3 - minCount3)
    return angleList, radiansPerCount1, radiansPerCount2, radiansPerCount3
