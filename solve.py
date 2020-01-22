# This file handles computing actual solutions by interfacing with the C++ solver.

from subprocess import Popen, PIPE
import time

N_THREADS = 12
N_SPLITS = 2
MILLIS = 25
N_WARMUPS = 100

# Simple Python interface to the rob-twophase CLI
class Solver:

    def __enter__(self):
        self.proc = Popen(
            ['./twophase', '-t', str(N_THREADS), '-s', str(N_SPLITS), '-m', str(MILLIS), '-w', str(N_WARMUPS), '-c'], 
            stdin=PIPE, stdout=PIPE
        )
        while 'Ready!' not in self.proc.stdout.readline().decode():
            pass # wait for everything to boot up
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.proc.terminate()

    def solve(self, facecube):
        if facecube == '':
            return None
        self.proc.stdin.write(('solve %s\n' % facecube).encode())
        self.proc.stdin.flush() # command needs to be received instantly
        tmp = self.proc.stdout.readline().decode() # either time taken or an error
        if 'error' in tmp:
            return tmp
        sol = self.proc.stdout.readline().decode()
        sol = sol[:(sol.index('(') - 1)] # delete appended solution length
        self.proc.stdout.readline() # clear "Ready!" message 
        return sol

    def scramble(self):
        self.proc.stdin.write('scramble\n'.encode())
        self.proc.stdin.flush()
        # Scrambling will never fail
        self.proc.stdout.readline() # facecube
        self.proc.stdout.readline() # time taken
        scramble = self.proc.stdout.readline().decode()[:-1] # strip '\n'
        self.proc.stdout.readline()
        return scramble


if __name__ == '__main__':
    import time

    print('Loading solver ...')
    with Solver() as solver:
        print('Done.')

        tick = time.time()
        print(solver.solve('FLBFURLBRBDUURFLULDRDRFRRLUFUBLDBDBDLUBLLFRFURBUDBDFDF'))
        print(time.time() - tick)

        tick = time.time()
        print(solver.scramble())
        print(time.time() - tick)

