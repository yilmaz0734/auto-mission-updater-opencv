import cv2
import numpy as np
imageFrame = cv2.imread("frames_autoc/frame25.jpg")
hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
h = hsvFrame[:,:,0]
s = hsvFrame[:,:,1]
v = hsvFrame[:,:,2]
print("h,s,v:",h,s,v)

#red_lower = np.array([140,75,180])
#red_upper = np.array([180,255,255])
red_lower,red_upper = np.array([136, 87, 160], np.uint8) , np.array([180, 255, 255], np.uint8)
red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
kernal = np.ones((5, 5), "uint8")
red_mask = cv2.dilate(red_mask, kernal)
res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                        mask = red_mask)

cv2.imwrite("Resd.jpg", res_red)