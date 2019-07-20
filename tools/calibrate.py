import pylab
import numpy
import cv2
import os
import sys

IMWIDTH = 1920
IMHEIGHT = 1080

DIR = sys.argv[1]
image_points = []
object_points = []
for fname in os.listdir(DIR):
    full_fname = os.path.join(DIR, fname)
    print(full_fname)
    d = pylab.loadtxt(full_fname, dtype=numpy.float32)
    image_points.append( d[:,1:3].copy() )
    object_points.append( d[:,3:6].copy() )

calib = cv2.calibrateCamera(object_points, image_points, (IMWIDTH, IMHEIGHT), None, None)

print("K=")
print(calib[1])
print("dist_coef=")
print(calib[2])

