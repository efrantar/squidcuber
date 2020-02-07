# Minimal utility for setting up the scan-positions.
# It is rather primitive and not at all user-friendly yet it gets the job done decently enough.

import os
import pickle
import sys
import time

import cv2
import numpy as np

from scan import *
from train import Rect, read_scanrects, extract_cols


GUI = 'Scan Setup'
COLOR = (0, 0, 0)
FILE = 'scan.rects'
SIZE = 5

scanner = Scanner('.')
scanner.connect()
scanner.start()

if len(sys.argv) > 1:
    image = cv2.imread(sys.argv[1])
else:
    scanner.save('shot.png')
    image = cv2.imread('shot.png')
    os.remove('shot.png')
image1 = image.copy()

rects = read_scanrects(FILE)
i = len(rects)
rects.append([])


def update_cam():
    global image
    global image1
    scanner.save('shot.png')
    image = cv2.imread('shot.png')
    image1 = image.copy()
    os.remove('shot.png')

def show_squares():
    global image1
    image1 = image.copy()
    for r in rects:
        for x, y, width, height in r:
            image1[y:(y + height), x:(x + width)] = COLOR

def show_nums():
    global image1
    image1 = image.copy()
    for i, r in enumerate(rects):
        for x, y, width, height in r:
            cv2.putText(
                image1, str(i), 
                (x + width // 2, y + height // 2), 
                cv2.FONT_HERSHEY_SIMPLEX, .5, COLOR
            )

def show_extracted():
    colors = extract_cols(image, rects[:-1]) # last rect is always empty

    global image1
    image1 = image.copy()
    for i, r in enumerate(rects):
        for x, y, width, height in r:
            image1[y:(y + height), x:(x + width)] = colors[i]

def show_matched():
    if len(rects) != 55:
        return

    tick = time.time()
    facecube = scanner.scan()
    print(time.time() - tick)

    if facecube == '':
        facecube = 'E' * 54
    print(facecube)

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
    for i, r in enumerate(rects):
        for x, y, width, height in r:
            image1[y:(y + height), x:(x + width)] = bgr[facecube[i]]


show = show_squares
i_edit = -1

def handle_click(event, x, y, flags, param):
    global i_edit

    if event == cv2.EVENT_LBUTTONDOWN:
        if i_edit == -1:
            for i, rs in enumerate(rects):
                for r in rs:
                    if r.x <= x <= r.x + r.width and r.y <= y <= r.y + r.height:
                        i_edit = i
                        rects[i].remove(r)
                        show()
                        return
            
            rects[-1].append(Rect(x - SIZE // 2, y - SIZE // 2, SIZE, SIZE))
        else:
            points[i_edit].append(Rect(x - SIZE // 2, y - SIZE // 2, SIZE, SIZE))
            i_edit = -1

        show()

cv2.namedWindow(GUI)
cv2.setMouseCallback(GUI, handle_click)
show()

while True:
    cv2.imshow(GUI, image1)
    key = cv2.waitKey(1) & 0xff
    if key == ord('i'):
        rects.append([])
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
scanner.disconnect()


del rects[-1]
with open(FILE, 'w') as f:
    for rs in rects:
        line = ' '.join([str(var) for r in rs for var in r])
        f.write(line + '\n')

