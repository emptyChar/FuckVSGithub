#!/usr/bin/env python

from email.headerregistry import HeaderRegistry
import rospy
import cv2
import time
import os
import mediapipe as mp
#import HandTrackingModule as htm
from std_msgs.msg import Int8
 
# This node publishes the count of finger to the action node
# Two nodes (HandTrackingModule and FingerCounter) are mixed Here


class fingerCounter():
    def __init__(self):
        rospy.init_node("cameraDistanceCalculator")
        
        self.fingerCounter_pub = rospy.Publisher("fingercounter",
                                            Int8,
                                            queue_size=1)

        self.mode = False
        self.maxHands = 2
        self.detectionCon = 0.75
        self.modelComplex = 1
        self.trackCon = 0.5
 
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex, 
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

        # self.detector = htm.handDetector(detectionCon=0.75)

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        # print(results.multi_hand_landmarks)
 
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo=0, draw=True):
 
        lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                # print(id, lm)
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                # print(id, cx, cy)
                lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
 
        return lmList

    # Main Method that performs the computation and publishes the fingercount
    def computation(self):
        wCam, hCam = 640, 480
 
        cap = cv2.VideoCapture(0)
        cap.set(3, wCam)
        cap.set(4, hCam)
        final_sum = []
        
        # folderPath = "FingerImages"
        # myList = os.listdir(folderPath)
        # print(myList)
        # overlayList = []
        # for imPath in myList:
        #     image = cv2.imread(f'{folderPath}/{imPath}')
        #     # print(f'{folderPath}/{imPath}')
        #     overlayList.append(image)
        
        # print(len(overlayList))
        pTime = 0
        
        # detector = htm.handDetector(detectionCon=0.75)
        
        tipIds = [4, 8, 12, 16, 20]
        
        while True:
            success, img = cap.read()
            img = self.findHands(img)
            lmList = self.findPosition(img, draw=False)
            # print(lmList)
        
            if len(lmList) != 0:
                fingers = []
        
                # Thumb
                if lmList[tipIds[0]][1] > lmList[tipIds[0] - 1][1]:
                    fingers.append(1)
                else:
                    fingers.append(0)
        
                # 4 Fingers
                for id in range(1, 5):
                    if lmList[tipIds[id]][2] < lmList[tipIds[id] - 2][2]:
                        fingers.append(1)
                    else:
                        fingers.append(0)
        
                # print(fingers)
                totalFingers = fingers.count(1)
                final_sum.append(totalFingers)

                if len(final_sum) == 50:
                    temp = Int8()
                    temp = max(final_sum, key=final_sum.count)
                    print(temp)
                    self.fingerCounter_pub.publish(temp)
                    final_sum.clear()
        
                # h, w, c = overlayList[totalFingers - 1].shape
                # img[0:h, 0:w] = overlayList[totalFingers - 1]
        
                cv2.rectangle(img, (20, 225), (170, 425), (0, 255, 0), cv2.FILLED)
                cv2.putText(img, str(totalFingers), (45, 375), cv2.FONT_HERSHEY_PLAIN,
                            10, (255, 0, 0), 25)
        
            cTime = time.time()
            fps = 1 / (cTime - pTime)
            pTime = cTime
        
            # cv2.putText(img, f'FPS: {int(fps)}', (400, 70), cv2.FONT_HERSHEY_PLAIN,
            #             3, (255, 0, 0), 3)
        
            cv2.imshow("Image", img)
            cv2.waitKey(1)
            

def main():
    input_count = fingerCounter()
    input_count.computation()

if __name__ == "__main__":
    main()
  

    
