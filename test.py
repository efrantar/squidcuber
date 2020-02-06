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

# robot.execute(convert("R' B2 (R' L) D2 (R L) U (R' L') D (F' B') U B U L' U (R' L) (F' B) L"))
# robot.execute(convert("(F B') (U' D) L U' (F B) U' B' (U' D') F R B D' (R' L') F2 L D2 L U2"))
# robot.execute(convert("U2 B' D B (R L') (U D) (F' B') D' R' U' B R' D' (F2 B2) U' (F2 B2) (U' D')"))

# robot.execute(convert("U R' F D' L B' U R' F D' L B' U R' F D' L B'"))
robot.execute(convert("U R F D L B U R F D L B U R F D L B"))
# robot.execute(convert("U' (R L) D' (F B) R' (U D) L' (F B) U' (R L) D' (F B) R' (U D) L' (F B)"))
# robot.execute(convert("U (R' L) D (F B') R (U' D) L (F B') U (R' L) D (F B') R (U' D) L (F' B)"))
# robot.execute(convert("U (R L) D (F B) R (U D) L (F B) U (R L) D (F B) R (U D) L (F B)"))
# robot.execute(convert("(U D) (R' L') (F B) (U' D') (R L) (F' B') (U D) (R' L') (F B) (U' D') (R L) (F' B')"))
# robot.execute(convert("(U D') (R' L) (F' B) (U D') (R' L) (F B') (U' D) (R L') (F' B) (U D') (R' L) (F' B)"))
# robot.execute(convert("(U D) (R L) (F B) (U D) (R L) (F B) (U D) (R L) (F B) (U D) (R L) (F B)"))

# robot.execute(convert("U2 R2 F2 D2 L2 B2 U2 R2 F2 D2 L2 B2 U2 R2 F2 D2 L2 B2"))
# robot.execute(convert("U2 (R2 L2) D2 (F2 B2) R2 (U2 D2) L2 (F2 B2) U2 (R2 L2) D2 (F2 B2) R2 (U2 D2) L2 (F2 B2)"))
# robot.execute(convert("(U2 D2) (R2 L2) (F2 B2) (U2 D2) (R2 L2) (F2 B2) (U2 D2) (R2 L2) (F2 B2) (U2 D2) (R2 L2) (F2 B2)"))

print(time.time() - tick)

