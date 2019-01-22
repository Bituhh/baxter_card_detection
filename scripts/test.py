#!/usr/bin/env python

from tf.transformations import quaternion_from_euler
from math import pi

r = quaternion_from_euler(0, pi/2, 0)
print(r)
