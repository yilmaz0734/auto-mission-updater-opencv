import cv2
import numpy as np
import glob

def take_number(path):
    number = path.split("/")[-1].split(".")[0][5:]
    number = int(number)
    return number

frameSize = (640, 480)

fnames=[]
for filename in glob.glob('frames/*.jpg'):
    fnames.append(filename)
out = cv2.VideoWriter('output_video.avi',cv2.VideoWriter_fourcc(*'MJPG'), len(fnames)/30, frameSize)
# sort fnames according to filenames
fnames.sort(key = take_number)



for i in fnames:
    img = cv2.imread(i)
    out.write(img)
out.release()