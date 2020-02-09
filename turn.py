# Generates the table with average transition times for all corner cutting situations
# based on data collected during previous solves.

import os
import pickle
from statistics import median
import numpy as np
from control import *

DIR = 'solves/'

data = []
for f in os.listdir(DIR):
    data.append(pickle.load(open(DIR + f, 'rb')))

agg_cut = [[[], []] for _ in range(11)]
# We also need ending move times for properly rating solutions
agg_end = [[[], []], [[], []]] # [is_axial][is_half]

for sol, times in data:
    for i in range(len(sol) - 1): # don't consider last move
        agg_cut[cut(sol[i], sol[i + 1])][int(is_half(sol[i]))].append(times[i])
    agg_end[int(is_axial(sol[-1]))][int(is_half(sol[-1]))].append(times[-1])

# Use medians so that rare lockups in recorded data don't mess up the values
med_cut = [[float('inf'), float('inf')] for _ in range(11)]
for i in range(len(agg_cut)):
    for j in range(len(agg_cut[i])):
        if len(agg_cut[i][j]) > 0:
            med_cut[i][j] = median(agg_cut[i][j])
med_end = [[float('inf'), float('inf')], [float('inf'), float('inf')]]
for i in range(2):
    for j in range(2):
        if len(agg_end[i][j]) > 0:
            med_end[i][j] = median(agg_end[i][j])

print(med_cut, med_end)
pickle.dump((med_cut, med_end), open('turn.times', 'wb'))

