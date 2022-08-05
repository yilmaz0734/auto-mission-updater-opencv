import cv2
import numpy as np
import glob

def take_number(path):
    number = path.split("/")[-1].split(".")[0][5:]
    number = int(number)
    return number

fnames=[]
for filename in glob.glob('frames/*.jpg'):
    fnames.append(filename)
out = cv2.VideoWriter('output_video.avi',cv2.VideoWriter_fourcc(*'MJPG'), len(fnames)/70, (1920,1080))
# sort fnames according to filenames
fnames.sort(key = take_number)
print(fnames)


for i in fnames:
    img = cv2.imread(i)
    out.write(img)
out.release()