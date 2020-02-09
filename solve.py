# This file handles computing actual solutions by interfacing with the C++ solver.

from subprocess import Popen, PIPE
import time

N_THREADS = 12
N_SPLITS = 2
MILLIS = 25
N_WARMUPS = 100
N_SOLS = 5

FACEID = {
    'U': 0, 'D': 1, 'R': 2, 'L': 3, 'F': 4, 'B': 5
}
COUNT = {
    '': 0, '2': 1, "'": 2
}

def move(s):
    return 4 * FACEID[s[0]] + COUNT[s[1:]]

def translate(s):
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

# Simple Python interface to the rob-twophase CLI
class Solver:

    def connect(self):
        self.proc = Popen(
            [
                './twophase', 
                '-t', str(N_THREADS), '-s', str(N_SPLITS), '-m', str(MILLIS), '-w', str(N_WARMUPS), '-c', '-n', str(N_SOLS)
            ], 
            stdin=PIPE, stdout=PIPE
        )
        while 'Ready!' not in self.proc.stdout.readline().decode():
            pass # wait for everything to boot up
        return self

    def disconnect(self):
        self.proc.terminate()

    def __enter__(self):
        return self.connect()

    def __exit__(self, exception_type, exception_value, traceback):
        self.disconnect()

    def solve(self, facecube):
        if facecube == '':
            return None
        self.proc.stdin.write(('solve %s\n' % facecube).encode())
        self.proc.stdin.flush() # command needs to be received instantly
        
        tmp = self.proc.stdout.readline().decode() # either time taken or an error
        if 'error' in tmp:
            self.proc.stdout.readline() # also need to clear "Ready!" here
            return None
        
        sols = [] # we return multiple solutions
        while True:
            sol = self.proc.stdout.readline().decode()
            if 'Ready!' in sol:
                break
            sol = ' '.join(sol.split(' ')[:-1]) # delete appended solution length
            sols.append(sol) 
        return sol

    def scramble(self):
        self.proc.stdin.write('scramble\n'.encode())
        self.proc.stdin.flush()

        # Scrambling will never fail
        self.proc.stdout.readline() # facecube
        self.proc.stdout.readline() # time taken

        scrambles = []
        while True:
            scramble = self.proc.stdout.readline().decode()
            if 'Ready!' in scramble:
                break        
            scramble = ' '.join(scramble.split(' ')[:-1])
            scrambles.append(scramble)
        return scrambles


if __name__ == '__main__':
    from control import *
    import time

    with Solver() as solver:
        scrambles = solver.scramble()

        tick = time.time()
        for scramble in scrambles:
            print(scramble, expected_time(optim_halfdirs(translate(scramble))))     
        print(time.time() - tick)

