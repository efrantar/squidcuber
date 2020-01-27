# Main program controlling the robot; not much is happening here, we just call the appropriate
# tools implemented in the other files.

import cv2
import pickle
import random
import time
import numpy as np

from control import *
from scan import *
from solve import *


def save_succ(frame, scans, facecube):
    cv2.imwrite('scans/succ/%s.png' % facecube, frame)
    pickle.dump(scans, open('scans/succ/%s.pkl' % facecube, 'wb'))

def save_fail(frame, scans):
    id = random.randint(0, 1000000)
    cv2.imwrite('scans/fail/%d.png' % id, frame)
    pickle.dump(scans, open('scans/fail/%d.pkl' % id, 'wb'))


with Solver() as solver:
    print('Solver initialized.')

    robot = Robot()
    print('Connected to robot.')

    size, points = pickle.load(open('setup.pkl', 'rb'))
    extractor = ColorExtractor(np.array(points), size)
    cam = DoubleCam()
    cam.start()
    print('Scanning set up.')

    print('Ready.') # we don't want to print this again and again while waiting for button presses
    while True: # polling is the most straight-forward way to check both buttons at once
        time.sleep(.05) # 50ms should be sufficient for a smooth experience
        if robot.scramble_pressed():
            cam.stop()
            scramble = solver.scramble()
            print(scramble)
            scramble = translate(scramble)
            start = time.time()
            robot.execute(scramble)
            print('Scrambled! %fs' % (time.time() - start))
            cam.start()
            continue
        elif not robot.solve_pressed():
            continue
        # Now actually start solving

        frame = cam.frame(stop=True)
        # NOTE: We start timing only after we have received a frame from the camera and start any processing.
        # While this might not be 100% conform to the Guiness World Record rules, I am (at least at this point)
        # not interested in optimizing the camera latency as I do not think this should be an integral part
        # of a cube-solving robot.
        start = time.time()
        print('Scanning ...')
        scans = extractor.extract_bgrs(frame)
        facecube = match_colors(scans)
        print('Solving ...')
        sol = solver.solve(facecube)
        print(time.time() - start)

        print(sol)
        if (sol is not None) and ('error' not in sol):
            print('Executing ...')
            robot.execute(translate(sol))
            print('Solved! %fs' % (time.time() - start))
            save_succ(frame, scans, facecube)
        else:
            print('Error.')
            save_fail(frame, scans)            

        cam.start()
        print('Ready.')

