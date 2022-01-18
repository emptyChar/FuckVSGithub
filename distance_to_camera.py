import cv2 
import imutils
import numpy as np
import argparse
import sys
import os
import time
from imutils.video import VideoStream
from cv2 import aruco as arc

marker_dict_type = arc.DICT_4X4_1000
arucoDict = arc.Dictionary_get(marker_dict_type)
arucoParams = arc.DetectorParameters_create()

#ap = argparse.ArgumentParser()
#ap.add_argument("-t","-type", required=True, help="tag of the marker to be detected")
#args = vars(ap.parse_args())
vs = VideoStream(src=0).start()
time.sleep(2.0)


CACHED_PTS = None
CACHED_IDS = None
Line_Pts = None
measure = None
while True:
    Dist = []
    image = vs.read()
    image = imutils.resize(image, width = 800)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        image, arucoDict, parameters = arucoParams
    )
    if len(corners) <= 0:
        if CACHED_PTS is not None:
            corners = CACHED_PTS

    if len(corners) > 0:
        CACHED_PTS = corners

        if ids is not None:
            ids = ids.flatten()
            CACHED_IDS = ids       
        else:
            if CACHED_IDS is not None:
                ids = CACHED_IDS
        if len(corners) < 2:
            if len(CACHED_PTS) >= 2:
                corners = CACHED_PTS
        for (markerCorner, markerId) in zip(corners, ids):
            print("[INFO] Marker detected")
            corners_abcd = markerCorner.reshape((4,2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners_abcd
            topRightPoint = (int(topRight[0]), int(topRight[1]))
            topLeftPoint = (int(topLeft[0]), int(topLeft[1]))
            bottomRightPoint = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeftPoint = (int(bottomLeft[0]), int(bottomLeft[1]))
            cv2.line(image, topLeftPoint, topRightPoint,(0, 255,0))
            cv2.line(image, topRightPoint, bottomRightPoint,(0, 255,0))
            cv2.line(image, bottomRightPoint, bottomLeftPoint,(0, 255,0))
            cv2.line(image, bottomLeftPoint, topLeftPoint,(0, 255,0))
            cx = int((topLeft[0] + bottomRight[0])//2) 
            cy = int((topLeft[1] + bottomRight[1])//2) 
            measure = abs(3.5/(topLeft[0] -cx))
            cv2.circle(image, (cx, cy), 4, (255,0,0), -1)
            cv2.putText(image, str(int(markerId)), (int(topLeft[0] -10),
            int(topLeft[1] -10)), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255))
            Dist.append((cx,cy))
            # print (arucoDict)
            if len(Dist) == 0:
                if Line_Pts is not None:
                    Dist = Line_Pts
            if len(Dist) == 2:
                Line_Pts = Dist
            if len(Dist) == 2:
                cv2.line(image, Dist[0], Dist[1], (255, 0, 255), 2)
                ed = ((Dist[0][0] - Dist[1][0])**2 +
                ((Dist[0][1] - Dist[1][1]) ** 2))**(0.5)
                cv2.putText(image, str(int(measure*(ed))) + "cm",
                (int(300), int(
                    300)), cv2.FONT_HERSHEY_COMPLEX,1, (0,0,255))
                cv2.imshow("[INFO] marker detected", image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
cv2.destroyAllWindows()
vs.stop()

#cameraMatrix = [[702.50846734 0 408.0060]
 #               [0 702.47748676 305.01744135]
  #              [0 0 1]]