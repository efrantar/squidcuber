from collections import namedtuple
from copy import deepcopy


class _Color:
    
    COUNT = 6

    U = 0
    R = 1
    F = 2
    D = 3
    L = 4
    B = 5

    CORNERS = [
        (U, R, F), (U, F, L), (U, L, B), (U, B, R),
        (D, F, R), (D, L, F), (D, B, L), (D, R, B)
    ]

    EDGES = [
        (U, R), (U, F), (U, L), (U, B),
        (D, R), (D, F), (D, L), (D, B),
        (F, R), (F, L), (B, L), (B, R)
    ]

    NAMES = ['U', 'R', 'F', 'D', 'L', 'B']
    
_PHYS_COLS = [
    'orange', 'yellow', 'blue', 'red', 'white', 'green'
]

class _Cubie:

    N_EDGES = 12
    N_CORNERS = 8
    
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

    # Map a facelet to the cubie it is on
    FROM_FACELET = [
        ULB, UB, UBR, UL, -1, UR, UFL, UF, URF,
        URF, UR, UBR, FR, -1, BR, DFR, DR, DRB,
        UFL, UF, URF, FL, -1, FR, DLF, DF, DFR,
        DLF, DF, DFR, DL, -1, DR, DBL, DB, DRB,
        ULB, UL, UFL, BL, -1, FL, DBL, DL, DLF,
        UBR, UB, ULB, BR, -1, BL, DRB, DB, DBL 
    ]

_N_FACELETS = 54

# Map a facelet to the position within its cubie
_FACELET_TO_POS = [
    0, 0, 0, 0, -1, 0, 0, 0, 0,
    1, 1, 2, 1, -1, 1, 2, 1, 1,
    1, 1, 2, 0, -1, 0, 2, 1, 1,
    0, 0, 0, 0, -1, 0, 0, 0, 0,
    1, 1, 2, 1, -1, 1, 2, 1, 1,
    1, 1, 2, 0, -1, 0, 2, 1, 1
]


# Implementation of full constraint matching & propagation for color assignment

class _Options:

    _Opt = namedtuple('Opt', ['cols', 'ori', 'cubie'])

    def __init__(self, cubiecols):
        self._opts = []
        for cubie, cols in enumerate(cubiecols):
            for ori in range(len(cols)):
                self._opts.append(_Options._Opt(
                    [cols[(i + ori) % len(cols)] for i in range(len(cols))], ori, cubie
                ))
        
        self._error = False
        self._colset = set()
        self._ori = -1
        self._cubie = -1

    @property
    def error(self):
        return self._error

    @property
    def colset(self):
        return self._colset

    @property
    def ori(self):
        return self._ori

    @property
    def cubie(self):
        return self._cubie

    def _update(self):
        self._error = len(self._opts) == 0
        if self._error:
            return

        for o in self._opts:
            self._colset &= set(o.cols)

        oris = set(o.ori for o in self._opts)
        self._ori = oris.pop() if len(oris) == 1 else -1 

        cubies = set(o.cubie for o in self._opts)
        self._cubie = cubies.pop() if len(cubies) == 1 else -1

    def _maybe_update(elim):
        def deco(self, *args):
            prelen = len(self._opts)
            elim(self, *args)
            if len(self._opts) != prelen:
                self._update()
        return deco
           
    @_maybe_update
    def has_poscol(self, pos, col):
        filtered = [o for o in self._opts if o.cols[pos] == col]
        if len(filtered) == 0:
            return False
        self._opts = filtered
        return True

    @_maybe_update
    def hasnot_col(self, col):
        self._opts = [o for o in self._opts if col in o.cols]

    @_maybe_update
    def has_ori(self, ori):
        self._opts = [o for o in self._opts if o.ori == ori]

    @_maybe_update
    def isnot_cubie(self, cubie):
        self._opts = [o for o in self._opts if o.cubie != cubie]

class _CubieBuilder:

    def __init__(self, cubiecols):
        self._n = len(cubiecols)
        self._noris = len(cubiecols[0])
     
        self._colcounts = [4] * _Color.COUNT
        self._colsets = [set() for _ in range(self._n)]
        self._oris = [-1] * self._n
        self._perm = [-1] * self._n
        self._par = -1

        self._opts = [_Options(cubiecols) for _ in range(self._n)]

        self._invcnt = 0
        self._orisum = 0
        self._aperm = 0
        self._aoris = 0

    def _assign_cubie(self, i):
        cubie = self._opts[i].cubie
        if self._perm[i] != -1 or cubie == -1:
            return False
        
        self._perm[i] = cubie
        for c in self._perm[:i]:
            if c != -1 and c > cubie:
                self._invcnt += 1
        for c in self._perm[(i + 1):]:
            if c != -1 and c < cubie:
                self._invcnt += 1
        self._aperm += 1
        if self._aperm == self._n:
            self._par = self._invcnt & 1 # permutation fully determined -> compute parity

        # Every cubie exists exactly once -> eliminate from all other options
        for j in range(self._n):
            if j != i:
                self._opts[j].isnot_cubie(cubie)

        return True

    def _assign_ori(self, i):
        ori = self._opts[i].ori
        if self._oris[i] != -1 or ori == -1:
            return False

        self._oris[i] = ori
        self._orisum += ori
        self._aoris += 1
        return True
        
    # This method returns whether or not an assignment is accepted (and thus any propagation should happen).
    # Optimally we would reject bad assignments only after propagation, however this would require deep
    # copying of the full builder-state which is pretty slow in Python. Furthermore, there are only very few
    # (if any) situations where this would be helpful so the current solutions should be good enough for now.
    def assign_col(self, cubie, pos, col):
        return self._opts[cubie].has_poscol(pos, col)

    def assign_par(self, par):
        self._par = par

    def propagate(self):
        change = True
        while change:
            change = False

            for cubie in range(self._n):
                if self._opts[cubie].error:
                    print('Error.', cubie)
                    return False

                diff = self._opts[cubie].colset - self._colsets[cubie]
                if len(diff) > 0:
                    self._colsets[cubie] |= diff
                    for col in diff:
                        self._colcounts[col] -= 1
                        if self._colcounts[col] == 0: # all cubies of certain color known
                            for i in range(self._n):
                                if col not in self._colsets[i]:
                                    self._opts[i].hasnot_col(col)
                    change = True

                change |= self._assign_ori(cubie)
                change |= self._assign_cubie(cubie)
               
                # Figure out last ori by parity
                if self._aoris == self._n - 1:
                    lastori = (self._noris - self._orisum % self._noris) % self._noris
                    for i in range(self._n):
                        if self._oris[i] == -1:
                            self._opts[i].has_ori(lastori)
                            self._assign_ori(i)
                            break
                    change = True

                # Figure out position of last two cubies by parity
                if self._par != -1 and self._aperm == self._n - 2:
                    tick = time.time()

                    i1, i2 = [i for i in range(self._n) if self._perm[i] == -1]
                    cubie1, cubie2 = sorted(set(i for i in range(self._n)) - set(self._perm))
                    
                    invcnt = 0
                    for j, cubie in enumerate(self._perm):
                        if cubie == -1:
                            continue
                        invcnt += int(j < i1 and cubie > cubie1)
                        invcnt += int(j > i1 and cubie < cubie1)
                        invcnt += int(j < i2 and cubie > cubie2)
                        invcnt += int(j > i2 and cubie < cubie2)

                    if ((self._invcnt + invcnt) & 1) != self._par:
                        i1, i2 = i2, i1 # flip cubie positions
                    self._opts[i1].isnot_cubie(cubie2) # safe because only `cubie1` and `cubie2` may be left
                    self._opts[i2].isnot_cubie(cubie1)
                    self._assign_cubie(i1)
                    self._assign_cubie(i2)
                    change = True
       
        return True


if __name__ == '__main__':
    import random

    CUBIECOLS = _Color.EDGES
    N_ORIS = len(CUBIECOLS[0])

    perm = [i for i in range(len(CUBIECOLS))]
    random.shuffle(perm)

    invs = 0
    for i in range(len(perm)):
        for j in range(i):
            if perm[j] > perm[i]:
                invs += 1
    par = invs & 1

    while True:
        oris = [random.randint(0, N_ORIS - 1) for _ in range(len(CUBIECOLS))]
        if sum(oris) % len(CUBIECOLS[0]) == 0:
            break

    print(perm, par)
    print(oris)

    assignments = [
        (cubie, pos, CUBIECOLS[perm[cubie]][(pos + oris[cubie]) % N_ORIS]) \
            for cubie in range(len(perm)) for pos in range(len(CUBIECOLS[0]))
    ]
    random.shuffle(assignments)

    builder = _CubieBuilder(CUBIECOLS)
    
    import time
    tick = time.time()

    builder = _CubieBuilder(CUBIECOLS)
    builder.assign_par(par)
    for cubie, pos, col in assignments:
        builder.assign_col(cubie, pos, col)
        builder.propagate()

    print(time.time() - tick)

    print(builder._perm, builder._par)
    print(builder._oris)

