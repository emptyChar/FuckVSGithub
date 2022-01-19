import cv2
import time
import os
import HandTrackingModule as htm

detector = htm.handDetector(detectionCon=0.75)
tipIds = [4, 8, 12, 16, 20]
pTime = 0
totalFingers = None

def counter_finger(cap):
    global pTime, totalFingers
    success, img = cap.read()
    img = detector.findHands(img)
    lmList = detector.findPosition(img, draw=False)

    if len(lmList) != 0:
        fingers = []

        # Thumb
        if lmList[tipIds[0]][1] > lmList[tipIds[0] - 1][1]:
            fingers.append(1)
        else:
            fingers.append(0)

        totalFingers = fingers.count(1)
 
        cv2.rectangle(img, (20, 225), (170, 425), (0, 255, 0), cv2.FILLED)
        cv2.putText(img, str(totalFingers), (45, 375), cv2.FONT_HERSHEY_PLAIN,
                    10, (255, 0, 0), 25)

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(img, f'FPS: {int(fps)}', (400, 70), cv2.FONT_HERSHEY_PLAIN,
                3, (255, 0, 0), 3)
 
    cv2.imshow("Image", img)
    cv2.waitKey(1)

    return totalFingers




if __name__ == "__main__":

    wCam, hCam = 640, 480
 
    cap = cv2.VideoCapture(0)
    cap.set(3, wCam)
    cap.set(4, hCam)
    fingers_count = []

    while(True):
        temp = counter_finger(cap)
        print(temp)
        # fingers_count.insert(0,counter_finger(cap))

        # if len(fingers_count) == 40:
        #     temp = max(fingers_count, key=fingers_count.count)
        #     print(temp)
