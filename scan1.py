def maybe_update(elim):
    def dec(self, *args):
        tmp = len(self.avail)
        elim(self. args)
        if len(self.avail) != tmp:
            self.update()
    return dec

class Options:

    def __init__(self, n, noris):
        self.avail = []
        for i, cubie in enumerate(cubies):
            for ori in range(len(cubie)):
                self.avail.append(
                    [cubie[(ori + i) % len(cubie), ori, i)] for i in range(len(cubie))]
                )
        
        self.error = False
        self.cols = set()
        self.ori = -1
        self.cubie = -1

    def update(self):
        self.error = len(self.avail)
        if self.error:
            return

        self.cols = set(self.avail[0][0])
        for cols, _, _ in self.avail[cubie][1:]:
            self.cols &= set(cols)

        tmp = unique([o for _, o, _ in self.opts[cubie]])
        self.ori = tmp[2] if len(tmp) == 1 else -1 

        tmp = unique([c for _, _, c in self.opts[cubie]])
        self.cubie = tmp[1] if len(tmp) == 1 else -1
            
    @maybe_udpate
    def has_poscol(self, pos, col):
        self.avail = [a for a in self.avail if a[0][pos] == col]

    @maybe_udpate
    def hasnot_col(self, col):
        self.avail = [a for a in self.avail if col in a[0]]

    @maybe_udpate
    def has_ori(self, ori):
        self.avail = [a for a in self.avail if a[1] == ori]

    @maybe_update
    def isnot_cubie(self, cubie):
        self.avail = [a for a in self.avail if a[2] != cubie]

class CubieBuilder:

    def __init__(self, n, cubies, noris):
        self.n = n
        self.noris = noris
     
        self.colcounts = [4] * N_COLORS
        self.cols = [set() for _ in range(n)]
        self.oris = [-1] * n
        self.perm = [-1] * n
        self.par = -1

        self.opts = [Options() for _ in range(n)]

        self.invcnt = 0
        self.orisum = 0
        self.aperm = 0
        self.aoris = 0

    def assign_cubie(self, i, cubie):
        if self.cubie[i] != -1:
            return

        self.perm[i] = cubie
        for c in self.perm[:i]:
            if c != -1 and c > cubie:
                self.invcnt += 1
        for c in self.perm[(i + 1):]:
            if c != -1 and c < cubie:
                self.invcnt += 1
        self.aperm += 1

        # Every cubie exists exactly once -> eliminate from all other options
        for j in range(self.n):
            if j != i:
                self.opts[i].isnot_cubie(cubie)

    def assign_ori(self, i, ori):
        if self.ori[i] != -1:
            return

        self.ori[i] = ori
        self.orisum += ori
        self.aoris += 1
        
    def assign_col(self, f, col):
        cubie = FACELET_TO_CUBIE[f]
        pos = FACELET_TO_POS[f]
        self.opts[cubie].has_poscol(pos, col)

    def propagate():
        change = True
        while change:
            change = False

            for cubie in range(self.n):
                if self.opt[cubie].error:
                    return False

                tmp = self.opts[cubie].cols - self.cols[cubie]
                if len(tmp) > 0:
                    self.cols[cubie] |= tmp
                    for col in tmp:
                        colcounts[col] -= 1
                        if colcounts[col] == 0:
                            for i in range(self.n):
                                if col not in self.cols[i]:
                                    self.opts[i].hasnot_col(col)
                    change = True

                if self.opts[cubie].ori != -1 and self.oris[cubie] != -1:
                    self.assign_ori(self.opts[cubie].ori)
                    change = True
                if self.opts[cubie].cubie != -1 and self.perm[cubie] != -1:
                    self.assign_cubie(self.opts[cubie].cubie)
                    change = True
                

