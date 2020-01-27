# Trains a KNN for assigning confidence scores to observed colors. For increased inference speed the model is 
# persisted in form of a full lookup table for all 16.7 million different BGR values.

import os
import pickle

import cv2
import numpy as np
from sklearn.neighbors import KNeighborsClassifier

from scan import *

size, points = pickle.load(open('setup.pkl', 'rb'))
extractor = ColorExtractor(np.array(points), size)

data = []
for f in os.listdir('data/'):
    labels = [c for c in f.split('.')[0]]
    img = cv2.imread('data/' + f)
    data.append((labels, extractor.extract_bgrs(img)))
print('Data loaded.')

# Red, orange, yellow, green, blue, red
HUES = np.array([0, 30, 60, 120, 240, 360]) / 360

# Transform hue space so that that the distance between all cube colors is the same 
def transform(hsv): 
    i = 1
    while i < 5:
        if hsv[0] < HUES[i]:
            break
        i += 1
    tmp = .2 * ((i - 1) + (hsv[0] - HUES[i - 1]) / (HUES[i] - HUES[i - 1]))
    return np.array([
        hsv[1] * np.cos(2 * np.pi * tmp), hsv[1] * np.sin(2 * np.pi * tmp), hsv[2]
    ])

# Convert to HSV and transform space
def preprocess(X):
    X = cv2.cvtColor(np.expand_dims(X, 0), cv2.COLOR_BGR2HSV)[0]
    X = X.astype(np.float)
    X[:, 0] /= 180
    X[:, 1:] /= 255
    return np.apply_along_axis(transform, 1, X)

y = np.concatenate([d[0] for d in data], axis=0)
X = np.concatenate([d[1] for d in data], axis=0)
X = preprocess(X)
print('Preprocessing done.')

model = KNeighborsClassifier(n_neighbors=int(.1 * X.shape[0]), n_jobs=-1)
model.fit(X, y)

allcols = np.zeros((256 ** 3, 3), dtype=np.uint8)
i = 0
for b in range(256):
    for g in range(256):
        for r in range(256):
            allcols[i] = [b, g, r]
            i += 1
allcols = preprocess(allcols)

print('Generating table ...')
table = model.predict_proba(allcols) * model.n_neighbors # will always be integer
table = table.astype(np.uint8) if model.n_neighbors <= 255 else table.astype(np.uint16)
pickle.dump(table, open('model.pkl', 'wb'))
print('Done.')

