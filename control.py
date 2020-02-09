# Robot control.

from cmd import *
from collections import namedtuple
import pickle
import time
import ev3


# We also consider inverted half-moves here (thus %/ 4 instead of 3)

def is_axial(move):
    return isinstance(move, tuple)

def is_half(move):
    if is_axial(move):
        return is_half(move[0]) or is_half(move[1])
    return (move % 4) % 2 == 1

def is_clock(move):
    return move % 4 <= 1

# Invert a half-turn
def inv2(m2):
    return (m2 // 4) * 4 + ((m2 + 2) % 4)


# All possible corner cutting situations
CUT = 0
ANTICUT = 1
AX_CUT1 = 2 # simple -> axial
AX_CUT2 = 3 # axial -> simple
AX_PARTCUT1 = 4
AX_PARTCUT2 = 5
AX_ANTICUT1 = 6
AX_ANTICUT2 = 7
AXAX_CUT = 8
AXAX_PARTCUT = 9
AXAX_ANTICUT = 10

def cut(m1, m2, inverted=False):
    if is_axial(m1) and not is_axial(m2):
        return cut(m2, m1, inverted=True) + 1
    if is_axial(m1) and is_axial(m2):
        return AXAX_CUT + (max(cut(m1, m2[0]), cut(m1, m2[1])) // 2 - 1)
    
    if not is_axial(m2):
        return CUT if is_clock(m1) != is_clock(m2) else ANTICUT
    
    m21, m22 = m2
    clock1 = is_clock(m21)
    clock2 = is_clock(m22)

    # Note that a special axial move only yields a simple incoming cut but not
    # an outcoming one
    if not inverted:
        if is_half(m21):
            if not is_half(m22):
                return CUT if clock1 != is_clock(m1) else ANTICUT
        else:
            if is_half(m22):
                return CUT if clock2 != is_clock(m1) else ANTICUT

    if clock1 == clock2:
        return AX_CUT1 if clock1 != is_clock(m1) else AX_ANTICUT1
    return AX_PARTCUT1


# [][0] for quarter- and [][1] for half-turns
WAITDEG = [
    [12, 52], # CUT
    [10, 50], # ANTICUT
    [20, 54], # AX_CUT1
    [24, 68], # AX_CUT2
    [22, 50], # AX_PARTCUT1
    [20, 56], # AX_PARTCUT2
    [12, 48], # AX_ANTICUT1
    [16, 60], # AX_ANTICUT2
    [24, 68], # AXAX_CUT
    [26, 68], # AXAX_PARTCUT
    [18, 64]  # AXAX_ANTICUT
]

# Slightly slower but extremely robust
WAITDEG_SAFE = [
    [14, 54], # CUT
    [11, 54], # ANTICUT
    [27, 72], # AX_CUT1
    [27, 72], # AX_CUT2
    [27, 63], # AX_PARTCUT1
    [27, 63], # AX_PARTCUT2
    [18, 66], # AX_ANTICUT1 
    [18, 66], # AX_ANTICUT2
    [27, 72], # AXAX_CUT
    [27, 72], # AXAX_PARTCUT
    [21, 72]  # AXAX_ANTICUT
]

# For half + quarter axial turns
SPECIAL_AX_WAITDEG = 5

# The waitdegs are not directly proportional to the actual execution speed. Thus
# it is better to do half-turn direction optimization and solution selection
# based on actual (collected) timing data
CUTTIMES, ENDTIMES = pickle.load(open('turn.times', 'rb'))

def expected_time(sol):
    time = 0
    for i in range(len(sol) - 1):
        time += CUTTIMES[cut(sol[i], sol[i + 1])][int(is_half(sol[i]))]
    time += ENDTIMES[int(is_axial(sol[i]))][int(is_half(sol[i]))]
    return time

# Determine optimal turning directions for half-turns with respect to corner cutting
def optim_halfdirs(sol):
    options = [[] for _ in range(len(sol))]
    for i in range(len(sol)):
        if is_axial(sol[i]):
            m1, m2 = sol[i]
            options[i].append(sol[i])
            if is_half(m1):
                options[i].append((inv2(m1), m2))
            if is_half(m2):
                options[i].append((m1, inv2(m2)))
            if is_half(m1) and is_half(m2):
                options[i].append((inv2(m1), inv2(m2)))
        else:
            options[i].append(sol[i])
            if is_half(sol[i]):
                options[i].append(inv2(sol[i]))

    # Dynamic programming to find the actual optimal maneuvers instead of just an approximation

    DP = [[float('inf')] * 4 for _ in range(len(sol))]
    PD = [[-1] * 4 for _ in range(len(sol))]

    DP[0] = [0] * 4
    for i in range(1, len(sol)):
        for j, op2 in enumerate(options[i]):
            for k, op1 in enumerate(options[i - 1]):
                tmp = DP[i - 1][k] + CUTTIMES[cut(op1, op2)][int(is_half(op1))]
                if tmp < DP[i][j]:
                    DP[i][j] = tmp
                    PD[i][j] = k

    j = 0
    for i in range(4):
        if DP[-1][i] < DP[-1][j]:
            j = i
    sol1 = [options[-1][j]]
    for i in range(len(sol) - 1, 0, -1):
        j = PD[i][j]
        sol1.append(options[i - 1][j])
    sol1.reverse()
    return sol1


Motor = namedtuple('Motor', ['brick', 'ports'])
DEGS = [54, 108, -54, -108] # double inversion from motor perspective + gearing

HOSTS = [
    '00:16:53:7F:36:D9',
    '00:16:53:4A:BA:BA',
    '00:16:53:40:CE:B6'
]

FACE_TO_MOTOR = [
    Motor(0, ev3.PORT_A + ev3.PORT_B), # U
    Motor(0, ev3.PORT_C + ev3.PORT_D), # D
    Motor(2, ev3.PORT_A + ev3.PORT_B), # R
    Motor(2, ev3.PORT_C + ev3.PORT_D), # L
    Motor(1, ev3.PORT_C + ev3.PORT_D), # F
    Motor(1, ev3.PORT_A + ev3.PORT_B)  # B
]

class Robot:

    def __init__(self):
        self.bricks = [
            ev3.EV3(protocol='Usb', host=host) for host in HOSTS
        ]

    def move(self, m, prev, next):
        motor = FACE_TO_MOTOR[m // 4]
        deg = DEGS[m % 4]

        if next is None:
            # NOTE: Cube can be considered solved once the final turn is < 45 degrees before completion
            waitdeg = abs(deg) - (27 - 1)
        else:
            waitdeg = WAITDEG[cut(m, next)][int(is_half(m))]

        rotate(self.bricks[motor.brick], motor.ports, deg, waitdeg)

    def move1(self, m, prev, next):
        m1, m2 = m
        motor1, motor2 = FACE_TO_MOTOR[m1 // 4], FACE_TO_MOTOR[m2 // 4]
        count1, count2 = m1 % 4, m2 % 4
        deg1, deg2 = DEGS[count1], DEGS[count2]
    
        if next is None:
            waitdeg = max(abs(deg1), abs(deg2)) - (27 - 1)
        else:
            waitdeg = WAITDEG[cut(m, next)][int(is_half(m))]

        # Half + quarter-turn case
        if (count1 & 1) != (count2 & 1):
            if (count2 & 1) != 0:
                return self.move1((m2, m1), prev, next)
            rotate2(
                self.bricks[motor1.brick], motor1.ports, motor2.ports, deg1, deg2,
                SPECIAL_AX_WAITDEG, waitdeg
            )
        else:
            # We always want to wait on the move with the worse in-cutting
            if prev is not None and WAITDEG[cut(prev, m1)] > WAITDEG[cut(prev, m2)]:
                return self.move1((m2, m1), prev, next)
            rotate1(self.bricks[motor1.brick], motor1.ports, motor2.ports, deg1, deg2, waitdeg)

    def execute(self, sol):
        if len(sol) == 0:
            return

        times = []
        for i in range(len(sol)):
            prev = sol[i - 1] if i > 0 else None
            next = sol[i + 1] if i < len(sol) - 1 else None
            
            tick = time.time()
            if is_axial(sol[i]):
                self.move1(sol[i], prev, next)
            else:
                self.move(sol[i], prev, next)
            times.append(time.time() - tick)
        return times # return for data collection purposes 

    def solve_pressed(self):
        return is_pressed(self.bricks[2], 0) # left button

    def scramble_pressed(self):
        return is_pressed(self.bricks[2], 1) # right button

