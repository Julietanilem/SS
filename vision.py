import cv2 as cv
import sys
import numpy as np
img = cv.imread(cv.samples.findFile("img1.png"))
if img is None:
    sys.exit("Could not read the image.")
cv.imshow("Display window", img)
k = cv.waitKey(0)
if k == ord("s"):
    cv.imwrite("starry_night.png", img)

img = np.random.randint(222, size=(2, 2,3))
gen = np.array(img ,dtype=np.uint8)
cv.imshow('Display window',gen)
cv.waitKey(0)
cv.destroyWindow('Display window')
print(gen[0])
for num in gen:
    print(gen[num])
