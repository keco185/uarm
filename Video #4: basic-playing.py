import cv2
import numpy as np
from uf.wrapper.swift_api import SwiftAPI
import time
# what's the maximum in positive that the arm can do?
ARM_Y_MAX = 220
RESCALE_FACTOR = 0.5
# expected diameter (in pixels) of puck
EXPECTED_DIAMETER = 50

cap = cv2.VideoCapture(1)
cap.set(3, int(1280*RESCALE_FACTOR))
cap.set(4, int(720*RESCALE_FACTOR))

# could not open port 'COM4': PermissionError(13, 'Access is denied.', None, 5)
accessed = False
while not accessed:
    try:
        swift = SwiftAPI()
        accessed = True
    except Exception as e:
        time.sleep(0.2)

print('device info: ')
print(swift.get_device_info())
swift.set_position(x=200, y=0, z=31, relative=False, speed=20000, wait=True)
swift.set_pump(True)


for i in range(15):
    _, frame = cap.read()

# find air hockey table
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
lower_table = np.array([0, 0, 128])
upper_table = np.array([180, 55, 255])
mask_table = cv2.inRange(hsv, lower_table, upper_table)
_, contours, _ = cv2.findContours(mask_table, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

fieldx = 0
fieldy = 0
fieldw = 0
fieldh = 0

# if table is found, use it, otherwise ask the user for ROI
if len(contours) > 0:
    bestContour = contours[0]
    # find largest contour to use as table
    for cnt in contours:
        if (cv2.contourArea(cnt) > cv2.contourArea(bestContour)):
            bestContour = cnt
    fieldx,fieldy,fieldw,fieldh = cv2.boundingRect(bestContour)
else:
    fieldx,fieldy,fieldw,fieldh = cv2.selectROI(frame)


# so it doesnt keep swinging to hit puck:
since_swing = 9999
while 1:
    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #hsv threshold for finding puck
    lower_puck = np.array([0, 90, 130])
    upper_puck = np.array([20, 255, 255])

    #hsv threshold for finding floor/background
    lower_bg = np.array([10, 83, 103])
    upper_bg = np.array([24, 255, 188])

    mask_puck = cv2.inRange(hsv, lower_puck, upper_puck)
    mask_bg = cv2.inRange(hsv, lower_bg, upper_bg)

    #if the pixel is true for the puck and false for the background, assume it is the puck
    res = cv2.bitwise_and(mask_puck, cv2.bitwise_not(mask_bg))

    _, contours, _ = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # if a potential puck is found, iterate through all possible options
    if (len(contours) > 0):
        bestContour = contours[0]
        (bestX,bestY),bestRadius = cv2.minEnclosingCircle(bestContour)
        bestRadius = int(bestRadius)
        bestCenter = (int(bestX),int(bestY))
        for cnt in contours:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            # if this object has a radius closest to actual puck radius, use it
            if abs(int(radius) - EXPECTED_RADIUS) < abs(bestRadius - EXPECTED_RADIUS):
                bestRadius = int(radius)
                bestCenter = (int(x),int(y))

        # compute puck x,y from 0,0 to 1,1 where 0,0 is top left of table 1,1 is bottom right
        puck_x = (float(bestCenter[0]) - float(fieldx)) / float(fieldw) * 100.0
        puck_y = (float(bestCenter[1]) - float(fieldy)) / float(fieldh) * 100.0

        # draw puck location and table location to frame
        cv2.circle(frame,bestCenter,bestRadius,(0,255,0),2)
        cv2.rectangle(frame,(fieldx,fieldy),(fieldx+fieldw,fieldy+fieldh),(0,255,0),2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame,"X: " + str(puck_x) + " Y: " + str(puck_y),(10,30), font,0.5,(0,255,0),2,cv2.LINE_AA)
        cv2.imshow('tracking', frame)
        print("Puck X: {}%    Puck Y: {}%".format(puck_x, puck_y))

        desired_arm_y = ((puck_y/100.0)*(ARM_Y_MAX*2))-ARM_Y_MAX

        print('DESIRED Y=', desired_arm_y, since_swing)
        swift.set_position(y=desired_arm_y, speed=1000000, wait=True)

        if puck_x > 80 and since_swing >= 20:
            swift.set_position(x=100, relative=True, speed=1000000, wait=True)
            time.sleep(0.3)
            swift.set_position(x=-100, relative=True, speed=1000000, wait=True)
            since_swing = 0

        since_swing += 1

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
