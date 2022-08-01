import cv2
import numpy as np
import math
import time
webcam = cv2.VideoCapture(0)
print("Webcam initialized, search for red carpet is started.")
start=time.time()
weight_list = []
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
    hull_list = []
    for pic, contour in enumerate(contours[:1]):
        break_out_flag = False
        area = cv2.contourArea(contour)
        hull = cv2.convexHull(contour, False)
        hull_list.append(hull)
        if(area > 1000):
            print("Red carpet found, the location will be saved.")
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w/2
            cy = y + h/2
            radius = w//2
            coordinates = (int(x+w/2), int(y+h/2))
            center_point = (int(imageFrame.shape[1])//2,int(imageFrame.shape[0])//2)
            xdist , ydist = coordinates[0] - center_point[0],center_point[1] - coordinates[1]
            pixel_distance = round(math.sqrt(xdist**2 + ydist**2),2)
            print("Saving process has been started...")
            roll,yaw,pitch=0,math.pi/4,0
            xreal,yreal=(4*xdist/radius),(4*ydist/radius)
            real_distance = round(math.sqrt(xreal**2 + yreal**2),2)
            east_d = math.cos(-yaw)*xreal - math.sin(-yaw)*yreal
            north_d = math.sin(-yaw)*xreal + math.cos(-yaw)*yreal
            dist_target=round(math.sqrt(east_d**2+north_d**2),2)
            variabler = cv2.isContourConvex(contour)
            print(center_point)
            red_carpet_loc=None
            info = """
x_d = {} y_d = {} p_d = {} \n
x_r = {} y_r = {} r_d = {} \n
pitch = {} yaw = {} roll = {} \n
east_t = {} north_t = {} t_d = {} \n
time = {} \n
p_l = {} \n
c_l = {} \n
variabler = {} 
            """.format(round(xdist,2),round(ydist,2),pixel_distance,
            round(xreal,2),round(yreal,2),real_distance,pitch,yaw,roll,
            round(east_d,2),round(north_d,2),dist_target,
            round(time.time()-start,3),
            [0,0,0],
            [0,0,0],
            variabler)
            #cv2.drawContours(imageFrame, hull_list,0, (0,0,0), 1, 8)
            #cv2.circle(imageFrame,(x+w//2,y+h//2),radius,(0,255,0),3)
            ellipse = cv2.fitEllipse(contour)
            cv2.ellipse(imageFrame,ellipse,(0,255,255),2)
            (xc,yc),(d1,d2),angle = ellipse
            print(xc,yc,d1,d1,angle)
            # draw circle at center
            xc, yc = ellipse[0]
            cv2.circle(imageFrame, (int(xc),int(yc)), 10, (255, 255, 255), -1)

            # draw vertical line
            # compute major radius
            rmajor = max(d1,d2)/2
            if angle > 90:
                angle = angle - 90
            else:
                angle = angle + 90
            print(angle)
            xtop = xc + math.cos(math.radians(angle))*rmajor
            ytop = yc + math.sin(math.radians(angle))*rmajor
            xbot = xc + math.cos(math.radians(angle+180))*rmajor
            ybot = yc + math.sin(math.radians(angle+180))*rmajor
            cv2.line(imageFrame, (int(xtop),int(ytop)), (int(xbot),int(ybot)), (0, 0, 255), 3)
            #cv2.putText(imageFrame,str(xtarget)+" "+str(ytarget)+" "+str(dist_target),((center_point[0]),(center_point[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.line(imageFrame,((center_point[0]),(center_point[1])),coordinates,(0,255,0),2)
            y0, dy = 7, 15
            for i, line in enumerate(info.split('\n')):
                y = y0 + i*dy
                cv2.putText(imageFrame, line, (0,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
    # Program Termination
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break