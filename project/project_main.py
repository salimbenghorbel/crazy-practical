import cflib.crtp
import time
import cv2
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
import pandas as pd
import numpy as np
from pandas.core.indexes import multi

import global_navigation as nav

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'


DEFAULT_HEIGHT = 0.4
DEFAULT_VELOCITY = 0.3


class Robot:
    # x+ is front
    # x- is back
    # y+ is left
    # y- is right

    # all length units are expressed in meters

    # Occupancy states
    TILE_UNEXPLORED = 0
    TILE_FREE = 1
    TILE_OBSTACLE = 2
    TILE_TAKEOFF = 3
    TILE_LANDING = 4

    # Robot states
    STATE_EXPLORATION_RIGHT = 0
    STATE_EXPLORATION_LEFT = 1
    STATE_EXPLORATION_RIGHT_BACK = 2
    STATE_EXPLORATION_LEFT_BACK = 3

    # 0.1m = 10cm precision
    GRID_PRECISION = 0.1
    DETECTION_THRESHOLD_SIDEWAY = 0.5
    DETECTION_THRESHOLD_Z = 0.5

    TAKEOFF_REGION_X = [0, 2]
    LANDING_REGION_X = [3, 6]

    Y_MIN = -1
    Y_MAX = 5

    def __init__(self, pc, multiranger, x=0, y=0, z=DEFAULT_HEIGHT):
        ind = pd.MultiIndex.from_arrays([[]] * 2, names=('x', 'y'))
        self.dynamic_occupancy = pd.Series(
            index=ind, name='occupancy_value', dtype='float64')
        self.x = x
        self.y = y
        self.z = z

        self.takeoff_position = [self.x, self.y]

        self.fill_dynamic_occupancy(self.x, self.y, self.TILE_TAKEOFF)
        self.state = self.STATE_EXPLORATION_RIGHT

    def truncate_to_grid_precision(self, coordinate):
        return int(coordinate/self.GRID_PRECISION)*self.GRID_PRECISION

    def fill_dynamic_occupancy(self, x, y, tile_state):
        x_truncated = self.truncate_to_grid_precision(x)
        y_truncated = self.truncate_to_grid_precision(y)

        # if element exists, update with new value, else create new item
        self.dynamic_occupancy.at[x_truncated, y_truncated] = tile_state

    def update_occupancy(self):
        self.x, self.y, self.z = self.pc.get_position()

        if self.multiranger.front < self.DETECTION_THRESHOLD_SIDEWAY:
            self.fill_dynamic_occupancy(
                self.x+self.multiranger.front, self.y, self.TILE_OBSTACLE)

        if self.multiranger.left < self.DETECTION_THRESHOLD_SIDEWAY:
            self.fill_dynamic_occupancy(
                self.x, self.y+self.multiranger.left, self.TILE_OBSTACLE)

        if self.multiranger.back < self.DETECTION_THRESHOLD_SIDEWAY:
            self.fill_dynamic_occupancy(
                self.x-self.multiranger.back, self.y, self.TILE_OBSTACLE)

        if self.multiranger.right < self.DETECTION_THRESHOLD_SIDEWAY:
            self.fill_dynamic_occupancy(
                self.x, self.y-self.multiranger.right, self.TILE_OBSTACLE)

        if self.multiranger.down < self.DETECTION_THRESHOLD_Z:
            if self.x <= self.TAKEOFF_REGION_X[0] and self.x >= self.TAKEOFF_REGION_X[1]:
                self.fill_dynamic_occupancy(self.x, self.y, self.TILE_TAKEOFF)
            elif self.x <= self.LANDING_REGION_X[0] and self.x >= self.LANDING_REGION_X[1]:
                self.fill_dynamic_occupancy(self.x, self.y, self.TILE_LANDING)
        else:
            self.fill_dynamic_occupancy(self.x, self.y, self.TILE_FREE)

    def put_pixel_in_map(self, row):
        x = row['level_0']
        y = row['level_1']
        pixel_label = row[0]

        if pixel_label == self.TILE_OBSTACLE:
            i = (x - self.x_min)/self.GRID_PRECISION
            j = (y - self.y_min)/self.GRID_PRECISION
            self.map[i, j] = 255

    def build_map(self):

        # First create array with detected obstacles.
        self.x_max = 1  # extract x_max
        self.x_min = 0  # extract x_min

        self.y_max = 1  # extract y_max
        self.y_min = 0  # extract y_min

        x_dim = (self.x_max - self.x_min)/self.GRID_PRECISION + 1
        y_dim = (self.y_max - self.y_min)/self.GRID_PRECISION + 1

        self.map = np.zeros((x_dim, y_dim))

        self.dynamic_occupancy.reset_index().apply(
            lambda row: self.put_pixel_in_map(row), axis=1)

        connected_points, _ = cv2.findContours(
            self.map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        connected_points_m = []
        for row in connected_points:
            row_m = []
            for pixel in row:
                x = pixel[0] * self.GRID_PRECISION + self.x_min
                y = pixel[1] * self.GRID_PRECISION + self.y_min
                row_m.append([x, y])
            connected_points_m.append(row_m)

        # build obstacles and build visibility graph.
        obstacles = nav.build_obstacles(
            connected_points_m, self.GRID_PRECISION)
        graph = nav.build_visgraph(obstacles)
        # apply A* to extract optimal path and target points coordinates.
        sp = nav.apply_astar(graph, [self.x, self.y], self.takeoff_position)
        self.all_target_points = []
        for i in range(0, len(sp)):
            self.all_target_points.append([sp[i].x, sp[i].y])
        self.target_point = self.all_target_points[0]

    def find_next_target_point(self):
        """
        find the next target points in the list f all target points
        """
        i = self.all_target_points.index(self.target_point)
        self.target_point[0] = self.all_target_points[i+1][0]
        self.target_point[1] = self.all_target_points[i+1][1]

    def behave_return(self):
        if self.truncate_to_grid_precision(self.x) == self.truncate_to_grid_precision(self.takeoff_position[0]) and \
                self.truncate_to_grid_precision(self.y) == self.truncate_to_grid_precision(self.takeoff_position[1]):

            self.pc.down(self.multiranger.down)
            return False
        elif self.multiranger.front < self.GRID_PRECISION or self.multiranger.back < self.GRID_PRECISION or \
                self.multiranger.left < self.GRID_PRECISION or self.multiranger.right < self.GRID_PRECISION:
            self.build_map()
        elif self.truncate_to_grid_precision(self.x) == self.truncate_to_grid_precision(self.target_point[0]) and \
                self.truncate_to_grid_precision(self.y) == self.truncate_to_grid_precision(self.target_point[1]):
            self.find_next_target_point()
        else:
            self.pc.go_to(self.x + self.GRID_PRECISION * np.sign(self.target_point[0] - self.x),
                          self.y + self.GRID_PRECISION * np.sign(self.target_point[1] - self.y))

        return True

    def behave_explore(self):
        self.x, self.y, self.z = self.pc.get_position()

        if self.multiranger.down < self.DETECTION_THRESHOLD_Z:
            if self.x <= self.LANDING_REGION_X[0] and self.x >= self.LANDING_REGION_X[1]:
                self.state = self.STATE_LANDING
        elif self.state == self.STATE_EXPLORATION_RIGHT:
            # map limit reached or obstacle on right sensor
            if self.y < self.Y_MIN or self.multiranger.right < self.GRID_PRECISION:
                if self.multiranger.front < self.GRID_PRECISION:  # if obstacle on front sensor, go left
                    self.state = self.STATE_EXPLORATION_RIGHT_BACK
                else:
                    # go forward and switch to left exploration
                    self.pc.forward(self.GRID_PRECISION)
                    self.state = self.STATE_EXPLORATION_LEFT
            else:
                self.pc.right(self.GRID_PRECISION)
        elif self.state == self.STATE_EXPLORATION_RIGHT_BACK:
            if self.multiranger.front > self.GRID_PRECISION:  # no obstacle on front sensor
                # go forward and switch to left exploration
                self.pc.forward(self.GRID_PRECISION)
                self.state = self.STATE_EXPLORATION_LEFT
            else:
                self.pc.left(self.GRID_PRECISION)
        if self.state == self.STATE_EXPLORATION_LEFT:
            # map limit reached or obstacle on left sensor
            if self.y > self.Y_MAX or self.multiranger.left < self.GRID_PRECISION:
                if self.multiranger.front < self.GRID_PRECISION:  # if obstacle on front sensor, go right
                    self.state = self.STATE_EXPLORATION_LEFT_BACK
                else:
                    # go forward and switch to right exploration
                    self.pc.forward(self.GRID_PRECISION)
                    self.state = self.STATE_EXPLORATION_RIGHT
            else:
                self.pc.left(self.GRID_PRECISION)
        elif self.state == self.STATE_EXPLORATION_LEFT_BACK:
            if self.multiranger.front > self.GRID_PRECISION:  # no obstacle on front sensor
                # go forward and switch to right exploration
                self.pc.forward(self.GRID_PRECISION)
                self.state = self.STATE_EXPLORATION_RIGHT
            else:
                self.pc.right(self.GRID_PRECISION)
        elif self.state == self.STATE_LANDING:
            self.pc.down(self.multiranger.down)
            return False
            # end program after landing

        # always return true before the landing
        return True


def main_sequence():
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(
                scf,
                x=0.0, y=0.0, z=0.0,
                default_velocity=DEFAULT_HEIGHT,
                default_height=DEFAULT_VELOCITY,
                controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
            with Multiranger(scf) as multiranger:

                # calibrate yaw
                pc._hl_commander.go_to(0, 0, DEFAULT_HEIGHT, 1.57, 2)
                time.sleep(2)
                pc._hl_commander.go_to(0, 0, DEFAULT_HEIGHT, 0, 2)
                time.sleep(2)

                x, y, z = pc.get_position()
                robot = Robot(pc, multiranger, x, y, z)

                while robot.behave_explore() == True:
                    robot.update_occupancy()
                    time.sleep(0.1)

                pc.up(DEFAULT_HEIGHT)

                robot.build_map()

                while robot.behave_return() == True:
                    robot.update_occupancy()
                    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    main_sequence()
