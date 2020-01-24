# Main program controlling the robot; not much is happening here, we just call the appropriate
# tools implemented in the other files.

import time

from control import *
from solve import *



with Solver() as solver:
    print('Solver initialized.')

    robot = Robot()
    print('Connected to robot.')

    print('Ready.') # we don't want to print this again and again while waiting for button presses
    while True: # polling is the most straight-forward way to check both buttons at once
        time.sleep(.05) # 50ms should be sufficient for a smooth experience
        if robot.scramble_pressed():
            scramble = solver.scramble()
            print(scramble)
            scramble = translate(scramble)
            start = time.time()
            robot.execute(scramble)
            print('Scrambled! %fs' % (time.time() - start))

