from control import *
import time

FACEID = {
    'U': 0, 'D': 1, 'R': 2, 'L': 3, 'F': 4, 'B': 5
}
COUNT = {
    '': 0, '2': 1, "'": 2
}

def move(s):
    return 4 * FACEID[s[0]] + COUNT[s[1:]]

def convert(s):
    sol = []
    
    splits = s.split(' ')
    i = 0

    while i < len(splits):
        m = splits[i]
        if '(' in m:
            m1 = m[1:]
            m2 = splits[i + 1][:-1]
            sol.append((move(m1), move(m2)))
            i += 1
        else:
            sol.append(move(m))
        i += 1

    return sol

robot = Robot()
print('Connected.')

tick = time.time()
robot.execute(convert("U R' F D' L B' U R' F D' L B' U R' F D' L B'"))
print(time.time() - tick)

