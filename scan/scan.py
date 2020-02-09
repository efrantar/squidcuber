from subprocess import Popen, PIPE


SCANDIR = 'scan'

class Scanner:

    def __init__(self, cwd):
        self.cwd = cwd
    
    def connect(self):
        self.proc = Popen(
            ['./scan'], stdin=PIPE, stdout=PIPE, cwd=self.cwd
        )
        while 'Ready!' not in self.proc.stdout.readline().decode():
            pass # wait for everything to boot up
        return self

    def disconnect(self):
        self.proc.terminate()

    def __enter__(self):
        return self.connect()

    def __exit__(self, exception_type, exceptioN_value, traceback):
        self.disconnect()

    def start(self):
        self.proc.stdin.write('start\n'.encode())
        self.proc.stdin.flush() # send command instantly
        self.proc.stdout.readline() # clear "Ready!"

    def stop(self):
        self.proc.stdin.write('stop\n'.encode())
        self.proc.stdin.flush()
        self.proc.stdout.readline()

    def scan(self):
        self.proc.stdin.write('scan\n'.encode())
        self.proc.stdin.flush()
        facecube = self.proc.stdout.readline().decode()[:-1] # strip '\n'
        self.proc.stdout.readline()
        return facecube if 'Error' not in facecube else ''

    def save(self, filename):
        self.proc.stdin.write(('save %s\n' % filename).encode())
        self.proc.stdin.flush()
        self.proc.stdout.readline()


if __name__ == '__main__':
    import time

    with Scanner('.') as scanner:
        print('Scanner ready.')

        scanner.start()
        scanner.stop()

        tick = time.time()
        print(scanner.scan())
        print(time.time() - tick)

