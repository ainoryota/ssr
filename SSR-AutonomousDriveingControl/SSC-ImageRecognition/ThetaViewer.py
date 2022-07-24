# python3
import cv2
import numpy as np
import random
import itertools
import math

vertex = 640
src_cx = 319
src_cy = 319
src_r = 283
src_cx2 = 1280 - src_cx

dst_map = np.array(
    list(itertools.product(range(vertex), range(vertex * 2))))

map_x = np.zeros((vertex, vertex * 2))
map_y = np.zeros((vertex, vertex * 2))
phi1 = math.pi * np.arange(vertex * 2) / vertex
theta1 = math.pi * np.arange(vertex) / vertex
X = np.matrix(np.cos(phi1)).T * np.sin(theta1)
Y = np.matrix(np.sin(phi1)).T * np.sin(theta1)
Z = np.cos(theta1)
phi2 = np.arccos(-X)
phi_cond = phi2 < np.pi / 2
theta2 = np.multiply(np.sign(
    Y), np.arccos(-np.divide(Z, np.sqrt(np.multiply(Y, Y) + np.multiply(Z, Z)))))

# 立体射影
r_ = np.where(phi_cond, np.tan((phi2) / 2), np.tan((np.pi - phi2) / 2))

cos_theta2 = np.cos(np.where(phi_cond, theta2, math.pi - theta2))
sin_theta2 = np.sin(np.where(phi_cond, theta2, math.pi - theta2))
map_x = (src_r * r_ * cos_theta2 + np.where(phi_cond, src_cx, src_cx2)).T
map_y = (src_r * r_ * sin_theta2 + src_cy).T
map_x = map_x.astype('float32')
map_y = map_y.astype('float32')


def align_imshow(image):
    return cv2.remap(cv2.resize(image, (1280, 720)), map_x, map_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)


cap = cv2.VideoCapture(-1)

while True:
    # VideoCaptureから1フレーム読み込む
    _, frame = cap.read()

    edframe = align_imshow(frame)
    cv2.imshow('Edited Frame', edframe)

    # キー入力を1ms待って、k が27（ESC）だったらBreakする
    k = cv2.waitKey(1)
    if k == 27:
        break

# キャプチャをリリースして、ウィンドウをすべて閉じる
cap.release()
cv2.destroyAllWindows()