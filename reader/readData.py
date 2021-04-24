import numpy as np


def readData(ser, width, height, regId=0xD0, regVal=0xD0):
    ser.write(bytearray([regId, regVal]))  # tells Arduino to begin capturing

    if regId == 0xD0 and regVal == 0xD0:
        dataY = ser.read(size=width * height)
        index = 0
        bitmap = list()
        for y in range(height):
            for x in range(width):
                Y = dataY[index]
                bitmap.append(Y)
                index += 1
        ser.write(bytearray([0x01, 0x01]))  # Stop reading data
        npimage = np.array(bitmap)
        arrImage = np.resize(npimage, [height, width])
        while ser.in_waiting > 0:
            ser.read()  # clear any remaining data
        data = getValues(ser)
        return arrImage, data


def getValues(ser):
    numPoints = 3  # number of datapoints to read
    dataList = [0] * numPoints  # initialize the list we will write to
    ser.write(bytearray([0x09, 0x09]))  # signal arduino to read pot values
    for i in range(0, numPoints):
        data = int(ser.readline().decode())
        dataList[i] = data
    return dataList
