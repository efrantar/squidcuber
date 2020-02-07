# Trains a KNN for assigning confidence scores to observed colors. For increased inference speed the model is 
# persisted in form of a full lookup table for all 16.7 million different BGR values.

import os
import pickle
from collections import namedtuple

import cv2
import numpy as np
from sklearn.neighbors import KNeighborsClassifier


COL_ORDER = {
    'U': 0, 'R': 1, 'F': 2, 'D': 3, 'L': 4, 'B': 5
}

Rect = namedtuple('Rect', ['x', 'y', 'width', 'height'])

def read_scanrects(filename):
    with open(filename, 'r') as f:
        rects = []
        for line in f.readlines():
            rects.append([])
            splits = line.strip().split(' ') # remove new-lines
            for i in range(0, len(splits), 4):
                rects[-1].append(Rect(*[int(n) for n in splits[i:(i + 4)]]))
        return rects 

def extract_cols(image, rects):
    scans = np.zeros((len(rects), image.shape[2]))
    for i, r in enumerate(rects):
        for x, y, width, height in r:
            tmp = image[y:(y + height), x:(x + width)]
            scans[i] += np.mean(tmp, axis=(0, 1))
        scans[i] /= len(rects[i])
    # Input has inaccuracies anyways + this is what makes a full conf-table possible
    return scans.astype(np.uint8)


if __name__ == '__main__': # we use some of the above functions in `setup.py`
    rects = read_scanrects('scan.rects')

    data = []
    for f in os.listdir('data/'):
        labels = [c for c in f.split('.')[0]]
        img = cv2.imread('data/' + f)
        data.append((labels, extract_cols(img, rects)))
    print('Data loaded. (%d)' % len(data))

    # Red, orange, yellow, green, blue, red
    HUES = np.array([0, 30, 60, 120, 240, 360]) / 360

    # Transform hue space so that the distance between all cube colors is roughly the same
    def transform(hsvs):
        sector = np.zeros(hsvs.shape[0], dtype=np.int)
        for i in range(1, 6):
            sector[(HUES[i - 1] <= hsvs[:, 0]) & (hsvs[:, 0] < HUES[i])] = i
        thue = .2 * ((sector - 1) + (hsvs[:, 0] - HUES[sector - 1]) / (HUES[sector] - HUES[sector - 1]))
        return np.column_stack([
            hsvs[:, 1] * np.cos(2 * np.pi * thue),
            hsvs[:, 1] * np.sin(2 * np.pi * thue),
            hsvs[:, 2]
        ])

    # Convert to HSV and transform space
    def preprocess(X):
        X = X.astype(np.uint8)
        X = cv2.cvtColor(np.expand_dims(X, 0), cv2.COLOR_BGR2HSV)[0]
        X = X.astype(np.float)
        X[:, 0] /= 180
        X[:, 1:] /= 255
        return transform(X)

    y1 = np.concatenate([d[0] for d in data], axis=0)
    y = np.zeros(y1.size, dtype=np.int)
    for col, id in COL_ORDER.items():
        y[y1 == col] = id

    X = np.concatenate([d[1] for d in data], axis=0)
    X = preprocess(X)
    model = KNeighborsClassifier(n_neighbors=int(.1 * X.shape[0]), n_jobs=-1)
    model.fit(X, y)
    print('Model learned.')

    b = np.concatenate([np.full(256 * 256, i) for i in range(256)])
    g = np.tile(np.concatenate([np.full(256, i) for i in range(256)]), 256)
    r = np.tile(np.arange(256), 256 * 256)
    allcols = np.column_stack([b, g, r])
    allcols = preprocess(allcols)

    print('Generating table ...')
    table = []
    for i, batch in enumerate(np.split(allcols, 256)):
        print(i)
        table.append(model.predict_proba(batch) * model.n_neighbors) # will always be integer
    table = np.concatenate(table, axis=0)
    table = table.astype(np.uint16)
    table.tofile('scan.tbl')
    print('Done.')

