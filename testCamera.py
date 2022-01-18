import cv2

cam = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1 ", cv2.CAP_GSTREAMER)

if cam.isOpened():
         cv2.namedWindow("demo", cv2.WINDOW_AUTOSIZE)
         while True:
             ret_val, img = cam.read()
             if not ret_val:
                  break
             cv2.imshow('demo',img)
             if cv2.waitKey(1) == ord('q'):
                  break
else:
    print ("camera open failed")