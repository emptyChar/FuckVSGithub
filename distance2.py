import numpy 
import cv2
import cv2.aruco as aruco
import os
import numpy as np

#cameraMatrix = [[702.50846734 0 408.0060]
#               [0 702.47748676 305.01744135]
#              [0 0 1]]
#  dist = [-5.22501437e-02 6.52677120e-02 3.44177484e-04 6.00359927e-04 -4.38803770e-01]

# Check for camera calibration data
cameraMatrix = np.array([[702.50846734, 0, 408.0060], [0, 702.47748676, 305.01744135], [0,0,1]])
distCoeffs = np.array([-5.22501437e-02, 6.52677120e-02, 3.44177484e-04, 6.00359927e-04, -4.38803770e-01])

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
#ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_1000) original
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_1000)

# Create grid board object we're using in our stream
board = aruco.GridBoard_create(
        markersX=2,
        markersY=2,
        markerLength=0.09,
        markerSeparation=0.01,
        dictionary=ARUCO_DICT)

# Create vectors we'll be using for rotations and translations for postures
rvecs, tvecs = None, None

cam = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1 ", cv2.CAP_GSTREAMER)


while(cam.isOpened()):
    # Capturing each frame of our video stream
    ret, QueryImg = cam.read()
    if ret == True:
        # grayscale image
        gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)

        # Detect Aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

        # Refine detected markers
        # Eliminates markers not part of our board, adds missing markers to the board
        corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
                image = gray,
                board = board,
                detectedCorners = corners,
                detectedIds = ids,
                rejectedCorners = rejectedImgPoints,
                cameraMatrix = cameraMatrix,
                distCoeffs = distCoeffs)   


        QueryImg = aruco.drawDetectedMarkers(QueryImg, corners, borderColor=(0, 0, 255))

    if ids is not None:
        try:
            rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners, 10.5, cameraMatrix, distCoeffs)                                           
            QueryImg = aruco.drawAxis(QueryImg, cameraMatrix, distCoeffs, rvec, tvec, 5)           
        except:
            print("Deu merda segue o baile")


        cv2.imshow('QueryImage', QueryImg)

    # Exit at the end of the video on the 'q' keypress
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()