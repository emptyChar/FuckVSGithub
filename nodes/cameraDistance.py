#!/usr/bin/env python

from cv2 import VideoCapture
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg       import Float64
import numpy 
import cv2
from cv2 import aruco
import os
import numpy as np
import math
import imutils


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CameraStuff():
    def __init__(self):
        rospy.init_node("cameraDistanceCalculator")
        self.bluerov_image = None
        self.bridge = CvBridge()
        # self.cam = VideoCapture(0)
        # MyCameraMatrix and dit coe
        #cameraMatrix = [[702.50846734 0 408.0060]
        #               [0 702.47748676 305.01744135]
        #              [0 0 1]]
        #  dist = [-5.22501437e-02 6.52677120e-02 3.44177484e-04 6.00359927e-04 -4.38803770e-01]


        # cameraMatrix of the robot:
        # [ 720.21893,    0.     ,  625.58763,
        #     0.     ,  721.62171,  346.05134,
        #     0.     ,    0.     ,    1.     ]
        
        # cameraMatrix = np.array([[720.21893, 0, 625.58763], [0, 721.62717, 346.05134], [0,0,1]])
        
        # distortionCoefficients of the robot:
        # 0.009464, 0.016208, 0.001510, 0.001006 0.000000
        #
        # distCoeffs = np.array([0.009464, 0.016208, 0.001510, 0.001006, 0.0])

        # # Check for camera calibration data
        self.cameraMatrix = cameraMatrix = np.array([[720.21893, 0, 625.58763], [0, 721.62717, 346.05134], [0,0,1]])
        self.distCoeffs = np.array([0.009464, 0.016208, 0.001510, 0.001006, 0.0])
        # Constant parameters used in Aruco methods
        self.ARUCO_PARAMETERS = aruco.DetectorParameters_create()
        
        #ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_1000) original
        self.ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_250)
        
        self.board = aruco.GridBoard_create(
                markersX=1,
                markersY=1,
                markerLength=0.1,
                markerSeparation=0.01,
                dictionary=self.ARUCO_DICT)

        self.camera_bluerov_sub = rospy.Subscriber("/bluerov/front_camera/image_color",
                                       Image,
                                       self.on_sub_bluerov,
                                       queue_size=1)

        self.markerPoint_pub = rospy.Publisher("markerPosition",
                                            Point,
                                            queue_size=1)

        self.markerAngle_pub = rospy.Publisher("markerAngle",
                                            Float64,
                                            queue_size=1)
     
    def on_sub_bluerov(self,msg):
        # cam = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1 ", cv2.CAP_GSTREAMER)
        # cam.set(cv2.CAP_PROP_AUTOFOCUS, 0) 
        rvecs, tvecs = None, None
        try:
            self.bluerov_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")          
        except CvBridgeError as e:
            print(e)          
        cv2.waitKey(1)        
        if self.bluerov_image.any != None:
            QueryImg = self.bluerov_image        
            #QueryImg = imutils.resize(QueryImg, width=600)
            gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)
            # alpha = 2 # Contrast control (1.0-3.0)
            # beta = 0 # Brightness control (0-100)
            # gray = cv2.convertScaleAbs(gray, alpha=alpha, beta=beta)
            # Detect Aruco markers
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.ARUCO_DICT, parameters=self.ARUCO_PARAMETERS)

            # Refine detected markers
            # Eliminates markers not part of our board, adds missing markers to the board
            # corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
            #         image = gray,
            #         board = self.board,
            #         detectedCorners = corners,
            #         detectedIds = ids,
            #         rejectedCorners = rejectedImgPoints,
            #         cameraMatrix = self.cameraMatrix,
            #         distCoeffs = self.distCoeffs)  
            QueryImg = aruco.drawDetectedMarkers(QueryImg, corners, borderColor=(0, 0, 255))
            
            if ids is not None:
                    try:
                        rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners, 10, self.cameraMatrix, self.distCoeffs)                                        
                        cameraPosition = Point(tvec[0][0][0] * 0.01, tvec[0][0][1] * 0.01, tvec[0][0][2] * 0.01)
                        self.markerPoint_pub.publish(cameraPosition)
                        self.markerAngle_pub.publish(Float64(rvec[0][0][2]))                     
                        cv2.putText(QueryImg, "%.1f X m -- %.0f deg" % ((tvec[0][0][0]) * 0.01, (rvec[0][0][2] / math.pi * 180)), (0, 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (244, 244, 244))
                        cv2.putText(QueryImg, "%.1f Y m -- %.0f deg" % ((tvec[0][0][1]) * 0.01, (rvec[0][0][2] / math.pi * 180)), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (244, 244, 244))
                        cv2.putText(QueryImg, "%.1f Z m -- %.0f deg" % ((tvec[0][0][2]) * 0.01, (rvec[0][0][2] / math.pi * 180)), (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (244, 244, 244))
                        
                        QueryImg = aruco.drawAxis(QueryImg, self.cameraMatrix, self.distCoeffs, rvec, tvec, 5)  
                        cv2.imshow("markerstandstuff", QueryImg)         
                    except:
                        print("Why you not working?")
            else:
                cameraPosition = Point(0, 0, 0.4)
                self.markerPoint_pub.publish(cameraPosition)
                self.markerAngle_pub.publish(Float64(0))

    
    def run(self):            
        while not rospy.is_shutdown():
            rospy.spin
            

def main():
    controller = CameraStuff()
    controller.run()


if __name__ == "__main__":
    main()

        

        






