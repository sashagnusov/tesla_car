import numpy as np
import cv2
import math

MARGIN = 270


class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __mul__(self, other):                           #Скалярное произведение векторов, символ - *
        return self.x * other.x + self.y * other.y

    def __mod__(self, other):                           #Векторное произведение векторов, символ - %
        return self.x * other.y - self.y * other.x


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def makeVectorDir(pointStart, pointDir):
    return Vector(pointDir.x - pointStart.x, pointDir.y - pointStart.y)


def getAngle_point(x, y, x_to, y_to, vector_dir):           # Принимает на вход x, y - наши координаты, x_to, y_to -
    vector_move = Vector(x_to - x, y_to - y)                # координаты точки финиша, vector_dir - вектор направления
    return (math.degrees(math.atan2(vector_dir * vector_move, vector_dir % vector_move)) + 270) % 360   # В градусах


def getAngle(pointStart, pointStop, pointDir):
    return getAngle_point(pointStart.x, pointStart.y, pointStop.x, pointStop.y, makeVectorDir(pointStart, pointDir))



m = [[0, 0], [0, -110]]
idm = 0

cap = cv2.VideoCapture(1)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    height = len(frame)
    width = len(frame[0])
    l = 40.
    cx = width//2
    cy = height//2
    ux = width//2
    uy = 0
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejectedImgpoints = cv2.aruco.detectMarkers(gray,dictionary)
    #print(res[0],res[1],len(res[2]))
    if len(corners) > 0:
        x0, y0 = corners[0][0][0]
        x1, y1 = corners[0][0][3]
        x2, y2 = int(x0 + l), int(y0)
        scalar = (x1 - x0) * (x2 - x0) + (y1 - y0) * (y2 - y0)
        vector = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        length = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2) * math.sqrt((x2 - x0) ** 2 + (y2 - y0) ** 2)
        sin = vector / length
        cos = scalar / length
        cx -= x0
        cy -= y0
        ux -= x0
        uy -= y0
        cx1 = cx * cos + cy * sin
        cy1 = -cx * sin + cy * cos
        ux1 = ux * cos + uy * sin
        uy1 = -ux * sin + uy * cos
        if(int(ids[0]) < len(m)):
            mx, my = m[int(ids[0])]
            cxa = cx1 - mx
            cya = cy1 - my
            uxa = ux1 - mx
            uya = uy1 - my           
            angle = getAngle(Point(cxa, cya), Point(m[idm][0], m[idm[1]), Point(uxa, uya))
            print(angle)
            #print(cxa, cya, uxa, uya)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.line(frame, (x0, 0), (x0, height), (255, 0, 0))
        cv2.line(frame, (0, y0), (height, y0), (255, 0, 0))
        cv2.line(frame, (x0, y0), (x1, y1), (0, 0, 255))
        cv2.line(frame, (x0, y0), (x2, y2), (0, 0, 255))

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()