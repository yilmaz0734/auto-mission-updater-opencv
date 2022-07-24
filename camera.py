import cv2
import numpy as np
import math
webcam = cv2.VideoCapture(1)
print("Webcam initialized, search for red carpet is started.")
while True:
    _, imageFrame = webcam.read()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    red_lower,red_upper = np.array([136, 87, 111], np.uint8) , np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    kernal = np.ones((5, 5), "uint8")
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                              mask = red_mask)
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    for pic, contour in enumerate(contours[:1]):
        break_out_flag = False
        area = cv2.contourArea(contour)
        if(area > 1000):
            print("Red carpet found, the location will be saved.")
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w/2
            cy = y + h/2
            radius = w//2
            coordinates = (int(x+w/2), int(y+h/2))
            center_point = (int(imageFrame.shape[1])//2,int(imageFrame.shape[0])//2)
            xdist , ydist = coordinates[0] - center_point[0],coordinates[1] - center_point[1]
            print("Saving process has been started...")
            roll,yaw,pitch=0,0,0
            xreal,yreal=(4*xdist/radius)/math.cos(roll),(4*ydist/radius)/math.cos(pitch)
            try:
                xtarget,ytarget=round(math.sqrt(xreal**2+yreal**2)*math.sin(yaw+math.atan(xreal/yreal)),2),round(math.sqrt(xreal**2+yreal**2)*math.cos(yaw+math.atan(xreal/yreal)),2)
            except:
                xtarget,ytarget=round(math.sqrt(xreal**2+yreal**2)*math.sin(yaw+math.pi/2),2),round(math.sqrt(xreal**2+yreal**2)*math.cos(yaw+math.pi/2),2)
            dist_target=round(math.sqrt(xtarget**2+ytarget**2),2)
            cv2.circle(imageFrame,(x+w//2,y+h//2),radius,(0,255,255),3)
            cv2.putText(imageFrame,str(xtarget)+" "+str(ytarget)+" "+str(dist_target),((center_point[0]),(center_point[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.line(imageFrame,((center_point[0]),(center_point[1])),coordinates,(0,255,0),2)
    # Program Termination
    cv2.imshow("Multiple Color Detection in Real-TIme", red_mask)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break