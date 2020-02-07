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


def save_succ(scanner, facecube):
    scanner.save('scans/succ/%s.png' % facecube)

def save_fail(scanner):
    scanner.save('scans/fail/%d.png' % random.randint(0, 1000000))


with Solver() as solver:
    print('Solver initialized.')
    
    with Scanner() as scanner:
        scanner.start()
        print('Scanning set up.')

        robot = Robot()
        print('Connected to robot.')

        print('Ready.') # we don't want to print this again and again while waiting for button presses
        while True: # polling is the most straight-forward way to check both buttons at once
            time.sleep(.05) # 50ms should be sufficient for a smooth experience
            if robot.scramble_pressed():
                scanner.stop()
                scramble = solver.scramble()
                print(scramble)
                scramble = translate(scramble)
                start = time.time()
                robot.execute(scramble)
                print('Scrambled! %fs' % (time.time() - start))
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
            sol = solver.solve(facecube)
            print(time.time() - start)

            print(sol)
            if (sol is not None) and ('error' not in sol):
                print('Executing ...')
                robot.execute(translate(sol))
                print('Solved! %fs' % (time.time() - start))
                save_succ(scanner, facecube)
            else:
                print('Error.')
                # save_fail(scanner)            

            scanner.start()
            print('Ready.')

