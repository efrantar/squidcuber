from collections import namedtuple
import copy
from heapq import *
import pickle
import time
import threading

import numpy as np
import cv2


N_COLORS = 6
N_EDGES = 12
N_CORNERS = 8
N_FACELETS = 54

# To do the constraint matching we have to repeat several definitions from the
# C++ solving algorithm.

NO_COL = -1
U = 0
R = 1
F = 2
D = 3
L = 4
B = 5

COL_NAMES = ['U', 'R', 'F', 'D', 'L', 'B']

URF = 0
UFL = 1
ULB = 2
UBR = 3
DFR = 4
DLF = 5
DBL = 6
DRB = 7

UR = 0
UF = 1
UL = 2
UB = 3
DR = 4
DF = 5
DL = 6
DB = 7
FR = 8
FL = 9
BL = 10
BR = 11

COLORS = ['orange', 'yellow', 'blue', 'red', 'white', 'green']

# Map a facelet to the cubie it is on
FACELET_TO_CUBIE = [
    ULB, UB, UBR, UL, -1, UR, UFL, UF, URF,
    URF, UR, UBR, FR, -1, BR, DFR, DR, DRB,
    UFL, UF, URF, FL, -1, FR, DLF, DF, DFR,
    DLF, DF, DFR, DL, -1, DR, DBL, DB, DRB,
    ULB, UL, UFL, BL, -1, FL, DBL, DL, DLF,
    UBR, UB, ULB, BR, -1, BL, DRB, DB, DBL 
]
# Map a facelet to the position within its cubie
FACELET_TO_POS = [
    0, 0, 0, 0, -1, 0, 0, 0, 0,
    1, 1, 2, 1, -1, 1, 2, 1, 1,
    1, 1, 2, 0, -1, 0, 2, 1, 1,
    0, 0, 0, 0, -1, 0, 0, 0, 0,
    1, 1, 2, 1, -1, 1, 2, 1, 1,
    1, 1, 2, 0, -1, 0, 2, 1, 1
]

# Generate all possible twisted configurations of the corners
CORNER_TWISTS = {
    (U, R, F), (U, F, L), (U, L, B), (U, B, R),
    (D, F, R), (D, L, F), (D, B, L), (D, R, B)
}
for c in list(CORNER_TWISTS):
    CORNER_TWISTS.add((c[1], c[2], c[0]))
    CORNER_TWISTS.add((c[2], c[0], c[1]))

CENTERS = [4, 13, 22, 31, 40, 49]


# `remove()` that does not throw if the element does not exis
def remove(l, e):
    try:
        l.remove(e)
    except:
        pass


class CubeBuilder:

    def __init__(self):
        self.colors = [NO_COL] * N_FACELETS
        self.ecols = [[NO_COL] * 2 for _ in range(N_EDGES)]
        self.ccols = [[NO_COL] * 3 for _ in range(N_CORNERS)]       
 
        self.eavail = [U, R, F, D, L, B] * 4
        self.eavail_part = [
            [R, L, F, B],
            [B, F, U, D],
            [U, D, R, L],
            [R, L, F, B],
            [U, D, F, B],
            [U, D, R, L]
        ]
        self.cavail = copy.deepcopy(self.eavail)
        self.cavail_part = [avail * 2 for avail in self.eavail_part]       

    def assign(self, facelet, col):
        if self.colors[facelet] != NO_COL:
            return True

        cubie = FACELET_TO_CUBIE[facelet]
        if (facelet % 9) % 2 == 1: # is on an edge
            if col not in self.edge_cols(cubie):
                print('elim', facelet, COLORS[col]) # NOTE: for debugging
                return False
            self.assign_edge(facelet, col)
        elif cubie != -1: # don't go here for centers
            if col not in self.corner_cols(cubie):
                print('elim', facelet, COLORS[col]) # NOTE: for debugging
                return False
            self.assign_corner(facelet, col)

        print('assign', facelet, COLORS[col]) # NOTE: for debugging
        self.colors[facelet] = col
        return True

    def assign_edge(self, facelet, col):
        edge = FACELET_TO_CUBIE[facelet]
        self.ecols[edge][FACELET_TO_POS[facelet]] = col

        remove(self.eavail, col)
        if len([c for c in self.ecols[edge] if c != NO_COL]) == 2: # edge fully assigned
            c1, c2 = self.ecols[edge]
            remove(self.eavail_part[c1], c2)
            remove(self.eavail_part[c2], c1)

    def assign_corner(self, facelet, col):
        corner = FACELET_TO_CUBIE[facelet]
        self.ccols[corner][FACELET_TO_POS[facelet]] = col

        remove(self.cavail, col)
        for c in self.ccols[corner]: # when we have more than 1 color of a corner
            if c != NO_COL and c != col:
                remove(self.cavail_part[col], c)
                remove(self.cavail_part[c], col)

    def edge_cols(self, edge):
        # If an edge already has one color simply return its available partners
        if self.ecols[edge][0] != NO_COL:
            return self.eavail_part[self.ecols[edge][0]]
        if self.ecols[edge][1] != NO_COL:
            return self.eavail_part[self.ecols[edge][1]]
        return self.eavail

    def corner_cols(self, corner):
        avail = set([c for c in range(N_COLORS)])
        count = 0
        i_missing = -1
        for i, c in enumerate(self.ccols[corner]):
            if c != NO_COL:
                avail &= set(self.cavail_part[c])
                count += 1
            elif i_missing < 0:
                i_missing = i

        if count == 1:
            return avail
        if count == 2: # use corner twists to limit the valid colors even more
            avail1 = []
            for c in avail:
                tmp = copy.copy(self.ccols[corner])
                tmp[i_missing] = c
                if tuple(tmp) in CORNER_TWISTS:
                    avail1.append(c)
            return avail1

        return self.cavail

    def facecube(self):
        return ''.join([COL_NAMES[c] for c in self.colors])


model = pickle.load(open('model.pkl', 'rb'))

Assignment = namedtuple('Assignment', ['conf', 'facelet', 'col', 'rank'])

COL_REMAP = [B, D, F, L, R, U] # map from model class to color ID

def match_colors(bgrs, fixed_centers=True):
    cube = CubeBuilder()

    counts = np.zeros((N_FACELETS, 6))
    for i in range(N_FACELETS):
        counts[i] = model[256 * (256 * bgrs[i, 0] + bgrs[i, 1]) + bgrs[i, 2]]
    counts = -counts # we want to do all sorting in decreasing order
    order = np.argsort(counts, axis=1)
    print(counts)

    assigned = 0
    if fixed_centers:
        for c, f in enumerate(CENTERS):
            cube.assign(f, c)
        assigned += 6

    heap = []
    for f in range(N_FACELETS):
        if fixed_centers and f in CENTERS:
            continue
        heap.append(
            Assignment(counts[f, order[f, 0]], f, COL_REMAP[order[f, 0]], 0)
        )
    heapify(heap)

    while assigned < N_FACELETS:
        ass = heappop(heap)
        if not cube.assign(ass.facelet, ass.col):
            f = ass.facelet
            i = ass.rank
            if i == N_COLORS - 1:
                return ''
            heappush(heap, Assignment(
                counts[f, order[f, i + 1]], f, COL_REMAP[order[f, i + 1]], i + 1 
            ))
        else:
            assigned += 1
    return cube.facecube()


class ColorExtractor:

    def __init__(self, points, size):
        self.points = points
        self.size = size

    def extract_bgrs(self, image):
        d = self.size // 2
        scans = np.zeros((self.points.shape[0], 3))

        for i in range(self.points.shape[0]):
            for x, y in self.points[i]:
                tmp = image[(y - d):(y + d), (x - d):(x + d)]
                scans[i] += np.mean(tmp, axis=(0, 1))
            scans[i] /= len(self.points[i]) # average over all scan points for face

        return scans.astype(np.uint8)


class DoubleCam:

    def __init__(self, uid=0, did=2):
        def cam(i):
            c = cv2.VideoCapture(i)
            c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            return c
        self.cams = [cam(uid), cam(did)]
        self.flocks = [threading.Lock(), threading.Lock()]
        self.slocks = [threading.Lock(), threading.Lock()]
        self.frames = [self.cams[0].read()[1], self.cams[1].read()[1]]
        self.rec = False

    def start(self):
        if self.rec:
            return
        def cap(i):
            while True:
                with self.slocks[i]:
                    if not self.rec:
                        break
                    _, frame = self.cams[i].read()
                with self.flocks[i]:
                    self.frames[i] = frame
        self.rec = True
        # Those threads should be terminated on program shutdown
        threading.Thread(target=lambda: cap(0), daemon=True).start()
        threading.Thread(target=lambda: cap(1), daemon=True).start()

    def stop(self):
        with self.flocks[0], self.flocks[1]: # Makes it easier to grab the `slocks`
            with self.slocks[0], self.slocks[1]: # ensure that we are not fetching any more frames on return
                self.rec = False

    # `stop` is used to not waste any processing time with frame decoding while computing a solution
    def frame(self, stop=False):
        with self.flocks[0], self.flocks[1]:
            if stop:
                with self.slocks[0], self.slocks[1]:
                    self.rec = False
            return np.concatenate(self.frames, axis=1)

