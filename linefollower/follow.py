#!/usr/bin/python3

import numpy as np
import cv2
from matplotlib import pyplot as plt
import os
import time

def handleImage(image):
  height, width, channels = image.shape
  image = image[:int(height * 0.7), :]

  hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

  lower = np.array([0,0,0])
  upper = np.array([125,125,125])

  mask = cv2.inRange(hsv, lower, upper)

  (thresh, blackwhite) = cv2.threshold(mask, 100, 255, cv2.THRESH_BINARY)
  blackwhite = cv2.medianBlur(blackwhite, 5)

  edges = cv2.Canny(blackwhite, 100, 200)
  lines = cv2.HoughLines(edges, 1, np.pi/180, 150)

  if(lines is not None):
    linecount = 0
    angle = 0
    for i in range(len(lines)):
      for rho,theta in lines[i]:
        linecount += 1
        angle += theta

        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        cv2.line(image,(x1,y1),(x2,y2),(255,125,125),20)

    avgangle = angle / linecount

    a = np.cos(avgangle)
    b = np.sin(avgangle)
    x0 = a
    y0 = b
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))
    y2 = height
    x2 = int(width / 2)
    cv2.line(image,(x1,y1),(x2,y2),(125,255,125),50)

  return image

# for root, subFolders, files in os.walk("dataset"):
#   for fil in files:
#     handle_pic("dataset/" + fil, "out/" + fil, False)

# image = cv2.imread("./dataset/notslot/image3.jpg")

cap = cv2.VideoCapture("./test.mp4")
while not cap.isOpened():
  cap = cv2.VideoCapture("./out.mp4")
  cv2.waitKey(1000)

pos_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
while True:
  flag, frame = cap.read()
  if flag:
    pos_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
    drawn = handleImage(frame)
    cv2.imshow('video', drawn)
    time.sleep(0.01)
  else:
    # The next frame is not ready, so we try to read it again
    cap.set(cv2.CAP_PROP_POS_FRAMES, pos_frame-1)
    # It is better to wait for a while for the next frame to be ready
    cv2.waitKey(1000)

  if cv2.waitKey(10) == 27:
    break
  if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
    # If the number of captured frames is equal to the total number of frames,
    # we stop
    break

plt.show()