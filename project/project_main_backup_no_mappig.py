import cflib.crtp
import time
import cv2
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from custom_multiranger import CustomMultiranger
import pandas as pd
import numpy as np
from pandas.core.indexes import multi

import global_navigation as nav
import pickle

from custom_position_hl_commander import CustomPositionHlCommander

import math

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'


DEFAULT_HEIGHT = 0.3
DEFAULT_VELOCITY = 0.3
PLATFORM_SIDE = 0.25
LAND_THRESHOLD = 0.09

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
    STATE_LANDING = 4
    STATE_FORWARD_LAND = 5
    STATE_RIGHT_LAND = 6
    STATE_LEFT_LAND = 7
    LANDING_MANEUVER_BACK = 8
    LANDING_MANEUVER_RIGHT = 9
    STATE_FORWARD = 10


    # 0.4m = 40cm precision
    GRID_PRECISION = 0.4
    DETECTION_THRESHOLD_SIDEWAY = 0.4
    DETECTION_THRESHOLD_Z = 0.5
    OBSTACLE_AVOIDANCE_THRESHOLD = 0.6
    FORWARD_STEP = 0.4

    SCANNER = 0.5

    TAKEOFF_REGION_X = [0, 0.5]
    #TAKEOFF_REGION_X = [0.5, 5]
    LANDING_REGION_X = [0.9, 5]

    Y_MIN = -0.2
    Y_MAX = 0.2

    def __init__(self, pc, multiranger, x=0, y=0, z=DEFAULT_HEIGHT):
        ind = pd.MultiIndex.from_arrays([[]] * 2, names=('x', 'y'))
        self.dynamic_occupancy = pd.Series(
            index=ind, name='occupancy_value', dtype='float64')
        self.x = x
        self.y = y
        self.z = z

        self.pc = pc
        self.multiranger = multiranger

        self.takeoff_position = [self.x, self.y]

        self.fill_dynamic_occupancy(self.x, self.y, self.TILE_TAKEOFF)
        self.state = self.STATE_EXPLORATION_RIGHT
        self.edge_displacement = None

        self.x_before_step = None
        self.y_before_step = None
        self.z_before_step = None

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
        x = row['x']
        y = row['y']
        pixel_label = row[0]

        if pixel_label == self.TILE_OBSTACLE:
            print(x, y, self.x_min, self.y_min)
            i = (x - self.x_min)/self.GRID_PRECISION
            j = (y - self.y_min)/self.GRID_PRECISION
            self.map[i, j] = 255

    def build_map(self):
        
        dynamic_occupancy_reset_index = self.dynamic_occupancy.reset_index()
        # First create array with detected obstacles.
        self.x_max = np.max(dynamic_occupancy_reset_index.x.values)
        self.x_min = np.min(dynamic_occupancy_reset_index.x.values)

        self.y_max = np.max(dynamic_occupancy_reset_index.y.values)
        self.y_min = np.min(dynamic_occupancy_reset_index.y.values)

        x_dim = int((self.x_max - self.x_min)/self.GRID_PRECISION + 1)
        y_dim = int((self.y_max - self.y_min)/self.GRID_PRECISION + 1)

        self.map = np.zeros((x_dim, y_dim),dtype=np.uint8)

        dynamic_occupancy_reset_index.apply(
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
        sp = nav.apply_astar(graph, [0, 0], self.takeoff_position)
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
        if self.truncate_to_grid_precision(self.x) == self.truncate_to_grid_precision(self.all_target_points[-1][0]) and \
                self.truncate_to_grid_precision(self.y) == self.truncate_to_grid_precision(self.all_target_points[-1][1]):

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

    def detect_edge(self):
        if self.up_list != []:
            self.x, self.y, self.z = self.pc.get_position()
            up_down = np.array(self.up_list) - np.array(self.down_list)
            print("up_down: ", up_down, "max - min = {}".format(max(up_down) - min(up_down)))
            if abs(max(up_down) - min(up_down)) > LAND_THRESHOLD and self.x >= self.LANDING_REGION_X[0] and self.x <= self.LANDING_REGION_X[1]:
                idx = (np.argmin(up_down) + np.argmax(up_down)) / 2
                distance_start_edge = idx / len(up_down) * self.FORWARD_STEP
                print("Distance to edge is", distance_start_edge)
                return distance_start_edge
            else:
                return None
        else:
            return None

    def detect_obstacles_while_moving(self):

        if self.state == self.STATE_EXPLORATION_RIGHT:
            if any(np.array(self.front_list < self.SCANNER)):

                # Get ids in list where obstacle near
                idx_list = np.where(np.array(self.front_list < self.SCANNER))
                # convert ids into y coordinate
                y_obstacles = self.y_before_step - idx_list[0]*9/100*self.FORWARD_STEP
                scan = np.array(self.front_list)
                # calculate x coordinate of obstacles basted on sensor value
                x_obstacles = self.x + scan[idx_list[0]]

                return y_obstacles, x_obstacles

        elif self.state == self.STATE_EXPLORATION_LEFT:
            if any(np.array(self.front_list < self.SCANNER)):

                idx_list = np.where(np.array(self.front_list < self.SCANNER))
                y_obstacles = self.y_before_step + idx_list[0]*9/100*self.FORWARD_STEP
                scan = np.array(self.front_list)
                x_obstacles = self.x + scan[idx_list[0]]

                return y_obstacles, x_obstacles

        elif self.state == self.STATE_FORWARD:
            if any(np.array(self.left_list < self.SCANNER)):

                idx_list = np.where(np.array(self.front_list < self.SCANNER))
                x_obstacles = self.x_before_step + idx_list[0] * 9 / 100 * self.FORWARD_STEP # step when going forward
                scan = np.array(self.front_list)
                y_obstacles = self.x + scan[idx_list[0]]

                return y_obstacles, x_obstacles

            if any(np.array(self.right_list < self.SCANNER)):

                idx_list = np.where(np.array(self.front_list < self.SCANNER))
                x_obstacles = self.x_before_step + idx_list[0] * 9 / 100 * self.FORWARD_STEP # step when going forward
                scan = np.array(self.front_list)
                y_obstacles = self.x - scan[idx_list[0]]

                return y_obstacles, x_obstacles
        else:
            pass

    def behave_explore(self):

        # storing the robot's position before movement
        if self.state != self.STATE_FORWARD_LAND and self.state != self.STATE_LEFT_LAND and self.state != self.STATE_RIGHT_LAND:
            self.x_before_step, self.y_before_step, self.z_before_step = self.pc.get_position()

        if self.x >= self.LANDING_REGION_X[0] and self.x <= self.LANDING_REGION_X[1]:
            self.DETECTION_THRESHOLD_SIDEWAY = 0.3
            self.OBSTACLE_AVOIDANCE_THRESHOLD = 0.3
            self.GRID_PRECISION = 0.15
            self.FORWARD_STEP = 0.15
        else:
            self.DETECTION_THRESHOLD_SIDEWAY = 0.6
            self.OBSTACLE_AVOIDANCE_THRESHOLD = 0.6
            self.GRID_PRECISION = 0.4
            self.FORWARD_STEP = 0.4
        
        self.x, self.y, self.z = self.pc.get_position()
        print("state: ", self.state, " position: {0} {1} {2}".format(self.x,self.y,self.z), " left: {0}, right {1}".format(self.multiranger.left, self.multiranger.right) )
        self.down_list = []
        self.up_list = []
        if self.state == self.STATE_EXPLORATION_RIGHT:
            # map limit reached or obstacle on right sensor
            try:
                obstacle_ahead = self.dynamic_occupancy.at[self.x,self.y-self.OBSTACLE_AVOIDANCE_THRESHOLD] == self.TILE_OBSTACLE
            except:
                obstacle_ahead = False
            if self.y < self.Y_MIN or self.multiranger.right < self.OBSTACLE_AVOIDANCE_THRESHOLD or obstacle_ahead:
                if self.multiranger.front < self.OBSTACLE_AVOIDANCE_THRESHOLD:  # if obstacle on front sensor, go left
                    self.state = self.STATE_EXPLORATION_RIGHT_BACK
                else:
                    # go forward and switch to left exploration
                    self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.forward(self.FORWARD_STEP)
                    self.edge_displacement = self.detect_edge()
                    if self.edge_displacement is not None:
                        self.state = self.STATE_FORWARD_LAND
                    else:
                        self.state = self.STATE_EXPLORATION_LEFT

            else:
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.right(self.GRID_PRECISION)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_RIGHT_LAND


        elif self.state == self.STATE_EXPLORATION_RIGHT_BACK:
            if self.multiranger.front > self.OBSTACLE_AVOIDANCE_THRESHOLD:  # no obstacle on front sensor
                # go forward and switch to left exploration
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.forward(self.FORWARD_STEP)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_FORWARD_LAND
                else:
                    self.state = self.STATE_EXPLORATION_LEFT
                self.state = self.STATE_EXPLORATION_LEFT
            else:
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.left(self.GRID_PRECISION)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_LEFT_LAND
        if self.state == self.STATE_EXPLORATION_LEFT:
            # map limit reached or obstacle on left sensor
            try:
                obstacle_ahead = self.dynamic_occupancy.at[self.x,self.y+self.OBSTACLE_AVOIDANCE_THRESHOLD] == self.TILE_OBSTACLE
            except:
                obstacle_ahead = False
            if self.y > self.Y_MAX or self.multiranger.left < self.OBSTACLE_AVOIDANCE_THRESHOLD or obstacle_ahead:
                if self.multiranger.front < self.OBSTACLE_AVOIDANCE_THRESHOLD:  # if obstacle on front sensor, go right
                    self.state = self.STATE_EXPLORATION_LEFT_BACK
                else:
                    # go forward and switch to right exploration
                    self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.forward(self.FORWARD_STEP)
                    self.edge_displacement = self.detect_edge()
                    if self.edge_displacement is not None:
                        self.state = self.STATE_FORWARD_LAND
                    else:
                        self.state = self.STATE_EXPLORATION_RIGHT
            else:
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.left(self.GRID_PRECISION)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_LEFT_LAND
        elif self.state == self.STATE_EXPLORATION_LEFT_BACK:
            if self.multiranger.front > self.OBSTACLE_AVOIDANCE_THRESHOLD:  # no obstacle on front sensor
                # go forward and switch to right exploration
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.forward(self.FORWARD_STEP)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_FORWARD_LAND
                else:
                    self.state = self.STATE_EXPLORATION_RIGHT
            else:
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.right(self.GRID_PRECISION)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_RIGHT_LAND

        elif self.state == self.STATE_FORWARD_LAND:
            self.x, self.y, self.z = self.pc.get_position()
            self.pc.go_to(self.x_before_step + self.edge_displacement + PLATFORM_SIDE/2, self.y + 0.5)
            self.state = self.LANDING_MANEUVER_RIGHT
            self.edge_displacement = None

        elif self.state == self.STATE_RIGHT_LAND:
            self.x, self.y, self.z = self.pc.get_position()
            self.pc.go_to(self.x + 0.5, self.y_before_step - self.edge_displacement - PLATFORM_SIDE/2)
            self.state = self.LANDING_MANEUVER_BACK
            self.edge_displacement = None

        elif self.state == self.STATE_LEFT_LAND:
            self.x, self.y, self.z = self.pc.get_position()
            self.pc.go_to(self.x + 0.5, self.y_before_step + self.edge_displacement + PLATFORM_SIDE/2)
            self.state = self.LANDING_MANEUVER_BACK
            self.edge_displacement = None

        elif self.state == self.LANDING_MANEUVER_BACK:
            self.pc.set_default_velocity(0.1)
            self.FORWARD_STEP = 0.15
            self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.back(self.FORWARD_STEP)
            self.LAND_THRESHOLD = 0.065
            self.edge_displacement = self.detect_edge()
            self.LAND_THRESHOLD = 0.09
            if self.edge_displacement is not None:
                self.x, self.y, self.z = self.pc.get_position()
                self.pc.go_to(self.x_before_step - self.edge_displacement - PLATFORM_SIDE/2, self.y_before_step )
                self.state = self.STATE_LANDING
                self.edge_displacement = None

        elif self.state == self.LANDING_MANEUVER_RIGHT:
            self.pc.set_default_velocity(0.1)
            self.FORWARD_STEP = 0.15
            self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.right(self.FORWARD_STEP)
            self.LAND_THRESHOLD = 0.065
            self.edge_displacement = self.detect_edge()
            self.LAND_THRESHOLD = 0.09
            if self.edge_displacement is not None:
                self.x, self.y, self.z = self.pc.get_position()
                self.pc.go_to(self.x_before_step, self.y_before_step - self.edge_displacement - PLATFORM_SIDE/2 )
                self.state = self.STATE_LANDING
                self.edge_displacement = None

        elif self.state == self.STATE_LANDING:
            return False
        # always return true before the landing
        self.x, self.y, self.z = self.pc.get_position()
        return True


def main_sequence():
    robot = Robot(None, None, 0, 0, 0)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(0.5)
        scf.cf.param.set_value('kalman.resetEstimation','1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation','0')

        with CustomMultiranger(scf, rate_ms=50) as multiranger:
            with CustomPositionHlCommander(
                    scf,
                    MotionCommander(scf, DEFAULT_HEIGHT),
                    multiranger,
                    x=0.0, y=0.0, z=0.0, 
                    landing_yaw = math.pi,
                    default_velocity=DEFAULT_HEIGHT,
                    default_height=DEFAULT_VELOCITY,
                    controller=CustomPositionHlCommander.CONTROLLER_MELLINGER) as pc:

                time.sleep(1)
                
                pc.set_default_velocity(0.1)
                time.sleep(1)
                
                robot.pc = pc
                robot.multiranger = multiranger
                x, y, z = pc.get_position()
                robot.x = x                
                robot.y = y
                robot.z = z
                
                while robot.behave_explore() == True:
                    robot.update_occupancy()
                
                time.sleep(2)
                pc._hl_commander.go_to(pc._x, pc._y, DEFAULT_HEIGHT, math.pi, 2)
                time.sleep(3)
                    
                with open('dynamic_occupancy.p', 'wb') as fp:
                    pickle.dump(robot.dynamic_occupancy, fp)
                
    time.sleep(2)
    robot.state = robot.STATE_EXPLORATION_RIGHT
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(0.5)
        scf.cf.param.set_value('kalman.resetEstimation','1')        
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation','0')

        scf.cf.param.set_value('kalman.initialX',str(robot.x))
        scf.cf.param.set_value('kalman.initialY',str(robot.y))
        time.sleep(0.1)
        with CustomMultiranger(scf, rate_ms=50) as multiranger:
            with CustomPositionHlCommander(
                    scf,
                    MotionCommander(scf, DEFAULT_HEIGHT),
                    multiranger,
                    x=0, y=0, z=0.0, 
                    landing_yaw = 0,
                    default_velocity=DEFAULT_HEIGHT,
                    default_height=DEFAULT_VELOCITY,
                    controller=CustomPositionHlCommander.CONTROLLER_MELLINGER) as pc:

                time.sleep(2)
                
                pc.set_default_velocity(0.1)
                

                robot.pc = pc
                robot.multiranger = multiranger
                x, y, z = pc.get_position()
                robot.x = x                
                robot.y = y
                robot.z = z
                while robot.behave_explore() == True:
                    robot.update_occupancy()
                with open('dynamic_occupancy.p', 'wb') as fp:
                    pickle.dump(robot.dynamic_occupancy, fp)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    main_sequence()
