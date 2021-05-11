import numpy as np
import matplotlib.pyplot as plt
# unit is in dm (1 dm = 0.1m)
x_size = 12
y_size = 12
grid_size = 1  # 1dm * 1dcm

class Map:
    def __init__(self, x_size, y_size, grid_size):
        self.xsize = x_size + 2  # Add extra cells for the borders
        self.ysize = y_size + 2
        self.grid_size = grid_size  # save this off for future use
        self.x_drone = 0
        self.y_drone = 0
        self.yaw = 0
        self.cells = np.zeros([x_size/grid_size,y_size/grid_size])

    def update_map(self, new_info):
        # 0 == unexplored, 1 == empty, 2 == obstacle, 3 == border, 4 == lift-off pad, 5 == landing-pad
        # update cells according to drone pos (define a radius for the proxy sensors, values 3,4 and 5 can be updated
        # only if drone is directly above cell of interest)