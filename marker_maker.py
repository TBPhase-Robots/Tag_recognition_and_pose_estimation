"""hello Github"""

import numpy as np
# import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

def marker_to_int(marker_num):
    img = aruco.drawMarker(aruco_dict, marker_num, 8)
    output = 0
    shift = 0

    for line in img[1:-1]:
        for pixel in line[1:-1]:
            if pixel == 255:
                output |= (1 << shift)
            shift += 1

    return output


fig = plt.figure()
nx = 1
ny = 1
for i in range(1, nx*ny+1):
    ax = fig.add_subplot(ny,nx, i)
    img = aruco.drawMarker(aruco_dict,i, 700)
    plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
    ax.axis("off")

print(marker_to_int(3))

plt.savefig("one_marker.pdf")
plt.show()
