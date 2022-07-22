import scipy.misc
import numpy as np
from skimage.draw import line_aa
import cv2
from skimage.draw import line #pip install scikit-image
def trapez(y,y0,w):
    return np.clip(np.minimum(y + 1 + w / 2 - y0, -y + 1 + w / 2 + y0),0,1)

def weighted_line(y1, x1, y2, x2, w, rmin=0, rmax=np.inf):
    # 傾き無限になるときはxとyを入れ替えて計算
    if abs(x2 - x1) < abs(y2 - y1) :
        # xとyを入れ替え
        xx, yy, val = weighted_line(x1, y1, x2, y2, w, rmin=rmin, rmax=rmax)
        return (yy, xx, val)

    # 点1x<点2xにする
    if x1 > x2 :
        return weighted_line(y2, x2, y1, x1, w, rmin=rmin, rmax=rmax)

    # The following is now always < 1 in abs
    slope = (y2 - y1) / (x2 - x1)

    # Adjust weight by the slope
    w *= np.sqrt(1 + np.abs(slope)) / 2

    # We write y as a function of x, because the slope is always <= 1
    # (in absolute value)
    x = np.arange(x1, x2 + 1, dtype=float)
    y = x * slope + (x2 * y1 - x1 * y2) / (x2 - x1)

    # Now instead of 2 values for y, we have 2*np.ceil(w/2).
    # All values are 1 except the upmost and bottommost.
    thickness = np.ceil(w / 2)
    yy = (np.floor(y).reshape(-1,1) + np.arange(-thickness - 1,thickness + 2).reshape(1,-1))
    xx = np.repeat(x, yy.shape[1])
    vals = trapez(yy, y.reshape(-1,1), w).flatten()
    
    yy = yy.flatten()

    # Exclude useless parts and those outside of the interval
    # to avoid parts outside of the picture
    #mask = np.logical_and.reduce((yy >= rmin, yy < rmax, vals > 0))

    return (yy.astype(int), xx.astype(int), vals)

h = 300
w = 300

x = 100
y = 300

img = np.zeros((h, w), dtype=np.uint8)
#rr, cc, val = weighted_line(0, 0,-200, -299,5)
rr, cc, val = weighted_line(0, 0,-200, -299,5)
#rr, cc = line(0, 0,-200, -299)
hoge = np.where((rr + y < h) & (cc + x < w) & (rr + y >= 0) & (cc + x >= 0))
img[rr[hoge] + y,cc[hoge] + x] = 255
print(img)
cv2.imshow('window', cv2.cvtColor(img, cv2.COLOR_GRAY2RGB))
cv2.waitKey()
