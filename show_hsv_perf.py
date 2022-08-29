import numpy as np
import math
import cv2
import sys
cap = cv2.VideoCapture('output_video.avi')

# Check if camera opened successfully

imageFrame = cv2.imread("frame9.jpg")
cv2.imwrite( "frame362.jpg",cv2.cvtColor(imageFrame,cv2.COLOR_RGB2BGR))
print(imageFrame.shape)
hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
red_lower = np.array([136,87,160])
red_upper = np.array([255,255,180])

red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
kernal = np.ones((5, 5), "uint8")
red_mask = cv2.dilate(red_mask, kernal)
res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                        mask = red_mask)

contours, hierarchy = cv2.findContours(red_mask,
                                    cv2.RETR_TREE,
                                    cv2.CHAIN_APPROX_SIMPLE)
contours = sorted(contours, key=cv2.contourArea, reverse=True)

cv2.drawContours(imageFrame, contours, -1, (0, 255, 0), 3)
cv2.imwrite("Red.jpg", res_red)



y0, dy = 7, 15
    
'''cv2.putText(imageFrame, line, (0,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
    cv2.circle(imageFrame,(x+w//2,y+h//2),int(radius),(0,255,255),3)
    cv2.putText(imageFrame,str(east_d)+" "+str(north_d)+" "+str(dist_target),((center_point[0]),(center_point[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.line(imageFrame,((center_point[0]),(center_point[1])),coordinates,(0,255,0),2)'''