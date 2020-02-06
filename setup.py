# Minimal utility for setting up the scan-positions.
# It is rather primitive and not at all user-friendly yet it gets the job done decently enough.

import os
import pickle
import sys
import time

import cv2
import numpy as np

from scan import *
from solve import *


DEBUG = True
if DEBUG:
    print('Debugging.')
    solver = Solver()
    solver.__enter__() # ugly but works

GUI = 'Scan Setup'
COLOR = (0, 0, 0)
FILE = 'setup.pkl'

cam = DoubleCam()
cam.start()
image = cv2.imread(sys.argv[1]) if len(sys.argv) > 1 else cam.frame()
image1 = image.copy()
size, points = pickle.load(open(FILE, 'rb')) if os.path.exists(FILE) else []
size2 = size // 2
i = len(points)
points.append([])


def update_cam():
    global image
    global image1
    image = cam.frame()
    image1 = image.copy()

def show_squares():
    global image1
    image1 = image.copy()
    for ps in points:
        for x, y in ps:
            image1[(y - size2):(y + size2), (x - size2):(x + size2), :] = COLOR

def show_nums():
    global image1
    image1 = image.copy()
    for i, ps in enumerate(points):
        for p in ps:
            cv2.putText(
                image1, str(i), (p[0] - 5, p[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, .5, COLOR
            )

def show_extracted():
    extractor = ColorExtractor(np.array(points[:-1]), size)
    tick = time.time()
    colors = extractor.extract_bgrs(image)
    print(colors)
    print(time.time() - tick)

    global image1
    image1 = image.copy()
    for i, ps in enumerate(points):
        for x, y in ps:
            image1[(y - size2):(y + size2), (x - size2):(x + size2), :] = colors[i, :]

def show_matched():
    if len(points) != 55:
        return
    cam.stop() # free CPU

    extractor = ColorExtractor(np.array(points[:-1]), size)
    tick = time.time()    
    colors = extractor.extract_bgrs(image)
    facecube = match_colors(colors)
    print(time.time() - tick)

    if facecube == '':
        facecube = 'E' * 54

    if DEBUG:
        if solver.solve(facecube) is None:
            facecube = 'E' * 54

    global image1
    image1 = image.copy()

    bgr = {
        'U': (0, 140, 255), # orange
        'R': (0, 255, 255), # yellow
        'F': (255, 0, 0), # blue
        'D': (0, 0, 139), # red
        'L': (255, 255, 255), # white
        'B': (0, 128, 0), # green
        'E': (128, 128, 128) # ERROR
    }
    for i, ps in enumerate(points):
        for x, y in ps:
            image1[(y - size2):(y + size2), (x - size2):(x + size2), :] = bgr[facecube[i]]

    cam.start() # capture again


show = show_squares
i_edit = -1

def handle_click(event, x, y, flags, param):
    global i_edit

    if event == cv2.EVENT_LBUTTONDOWN:
        if i_edit == -1:
            for i, ps in enumerate(points):
                for p in ps:
                    if p[0] - size2 <= x <= p[0] + size2 and p[1] - size2 <= y <= p[1] + size2:
                        i_edit = i
                        points[i].remove(p)
                        show()
                        return
            
            points[-1].append((x, y))
        else:
            points[i_edit].append((x, y))
            i_edit = -1

        show()

cv2.namedWindow(GUI)
cv2.setMouseCallback(GUI, handle_click)
show()

while True:
    cv2.imshow(GUI, image1)
    key = cv2.waitKey(1) & 0xff
    if key == ord('i'):
        points.append([])
        i += 1
    elif key == ord('c'):
        update_cam()
        show()
    elif key == ord('s'):
        show = show_squares
        show()
    elif key == ord('n'):
        show = show_nums
        show()
    elif key == ord('e'):
        show = show_extracted
        show()
    elif key == ord('m'):
        show_matched()
    elif key == ord('q'):
        break

cv2.destroyAllWindows()


del points[-1]
pickle.dump(points, open(FILE, 'wb'))

