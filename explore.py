import os
import pickle

import cv2
from mpl_toolkits.mplot3d import Axes3D 
import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import KNeighborsClassifier

from scan import *

PATH = 'scans/succ/'

scans = []
for file in os.listdir(PATH):
    if file.endswith('.pkl'):
        scans.append((np.array([c for c in file.split('.')[0]]), pickle.load(open(PATH + file, 'rb'))))

cols = np.concatenate([s[0] for s in scans], axis=0)
bgrs = np.concatenate([s[1] for s in scans], axis=0)

hsvs = cv2.cvtColor(np.expand_dims(bgrs, 0), cv2.COLOR_BGR2HSV)[0]
hsvs = hsvs.astype(np.float)
hsvs[:, 0] /= 180
hsvs[:, 1:] /= 255
trans = np.apply_along_axis(transform, 1, hsvs)

bgrs1 = bgrs[-(2 * 54):]
trans1 = trans[-(2 * 54):]
cols1 = cols[-(2 * 54):]
bgrs = bgrs[:-(2 * 54)]
trans = trans[:-(2 * 54)]
cols = cols[:-(2 * 54)]

print(len(trans), len(cols))

model = KNeighborsClassifier(n_neighbors=50)
model.fit(trans, cols)

import time
tick = time.time()
print(model.predict_proba(trans1))
print(time.time() - tick)

print(model.predict_proba(trans1)[model.predict(trans1) != cols1])
print(model.predict(trans1)[model.predict(trans1) != cols1])

fig = plt.figure()
ax = fig.gca(projection='3d')

angles = np.linspace(0, 2 * np.pi, 1000)
ax.plot(np.cos(angles), np.sin(angles), np.ones(angles.size))

for i in np.where(model.predict(trans1) != cols1)[0]:
    ax.scatter(
        trans1[i, 0], trans1[i, 1], trans1[i, 2],
        marker='o', color=bgrs1[i, ::-1] / 255
    )

plt.show()

