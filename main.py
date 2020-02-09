# Main program controlling the robot; not much is happening here, we just call the appropriate
# tools implemented in the other files.

from datetime import datetime
import pickle
import random
import time

import cv2
import numpy as np

from control import *
from scan.scan import *
from solve import *


def save_scan(scanner, facecube):
    scanner.save('scan/data/%s.png' % facecube)

def save_times(sol, times):
    f = datetime.now().strftime('%y%m%d%H%M%S')
    pickle.dump((sol, times), open('solves/%s.pkl' % f , 'wb'))


# Select the fastest of the solutions returned by the solver
def sel_best(sols):
    sols = [optim_halfdirs(translate(sol)) for sol in sols]
    times = [expected_time(sol) for sols in sol]
    best = 0
    for i in range(1, len(sols)):
        if times[i] < times[best]:
            best = i
    return sols[best]


with Solver() as solver:
    print('Solver initialized.')
    
    with Scanner(SCANDIR) as scanner:
        scanner.start()
        print('Scanning set up.')

        robot = Robot()
        print('Connected to robot.')

        print('Ready!') # we don't want to print this again and again while waiting for button presses
        while True: # polling is the most straight-forward way to check both buttons at once
            time.sleep(.05) # 50ms should be sufficient for a smooth experience
            if robot.scramble_pressed():
                scanner.stop()
                scramble = sel_best(solver.scramble())
                start = time.time()
                print('Executing ...')
                times = robot.execute(scramble)
                print('Scrambled! %fs' % (time.time() - start))
                save_times(scramble, times)
                scanner.start()
                continue
            elif not robot.solve_pressed():
                continue
            # Now actually start solving

            scanner.stop() # don't waste any CPU resources fetching further images
            # NOTE: We start timing only after we have received a frame from the camera and start any processing.
            # While this might not be 100% conform to the Guiness World Record rules, I am (at least at this point)
            # not interested in optimizing the camera latency as I do not think this should be an integral part
            # of a cube-solving robot.
            start = time.time()
            print('Scanning ...')
            facecube = scanner.scan()
            print('Solving ...')
            sol = sel_best(solver.solve(facecube))

            if (sol is not None) and ('error' not in sol):
                print('Executing ...')
                times = robot.execute(sol)
                print('Solved! %fs' % (time.time() - start))
                save_times(sol, times)
                save_scan(scanner, facecube)
            else:
                print('Error.')        

            scanner.start()
            print('Ready!')

