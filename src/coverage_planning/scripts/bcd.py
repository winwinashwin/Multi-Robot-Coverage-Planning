#!/usr/bin/env python3
import enum

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image

rospy.init_node("boustrophedon", anonymous=True)

grid = rospy.wait_for_message("map", OccupancyGrid)  # type: OccupancyGrid

mat = np.ones((grid.info.height, grid.info.width))
for i in range(grid.info.height):
    for j in range(grid.info.width):
        mat[i][j] = grid.data[j + i * grid.info.width]
mat = 100 - mat
mat[mat > 100] = 0
mat[mat > 0] = 255


im = Image.fromarray(mat)
im.show()
