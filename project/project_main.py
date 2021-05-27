import cflib.crtp
import time
import cv2
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from cflib.utils.multiranger import Multiranger
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
    STATE_LANDING_BACK = 5

    # 0.4m = 40cm precision
    GRID_PRECISION = 0.4
    DETECTION_THRESHOLD_SIDEWAY = 0.4
    DETECTION_THRESHOLD_Z = 0.5
    OBSTACLE_AVOIDANCE_THRESHOLD = 0.6
    FORWARD_STEP = 0.4


    TAKEOFF_REGION_X = [0, 0.5]
    #TAKEOFF_REGION_X = [0.5, 5]
    LANDING_REGION_X = [0.5, 5]

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

        x_dim = int((self.x_max - self.x_min)/self.GRID_PRECISION + 1)
        y_dim = int((self.y_max - self.y_min)/self.GRID_PRECISION + 1)

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
        if self.x >= self.LANDING_REGION_X[0] and self.x <= self.LANDING_REGION_X[1]:
            self.DETECTION_THRESHOLD_SIDEWAY = 0.3
            self.OBSTACLE_AVOIDANCE_THRESHOLD = 0.3
            self.GRID_PRECISION = 0.1
            self.FORWARD_STEP = 0.1
        else:
            self.DETECTION_THRESHOLD_SIDEWAY = 0.6
            self.OBSTACLE_AVOIDANCE_THRESHOLD = 0.6
            self.GRID_PRECISION = 0.4
            self.FORWARD_STEP = 0.4
        
        self.x, self.y, self.z = self.pc.get_position()
        print("state: " , self.state, " position: {0} {1} {2}".format(self.x,self.y,self.z), " left: {0}, right {1}".format(self.multiranger.left, self.multiranger.right))
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
                    self.up_list, self.down_list = self.pc.forward(self.FORWARD_STEP)
                    self.state = self.STATE_EXPLORATION_LEFT
            else:
                self.up_list, self.down_list = self.pc.right(self.GRID_PRECISION)
        elif self.state == self.STATE_EXPLORATION_RIGHT_BACK:
            if self.multiranger.front > self.OBSTACLE_AVOIDANCE_THRESHOLD:  # no obstacle on front sensor
                # go forward and switch to left exploration
                self.up_list, self.down_list = self.pc.forward(self.FORWARD_STEP)
                self.state = self.STATE_EXPLORATION_LEFT
            else:
                self.up_list, self.down_list = self.pc.left(self.GRID_PRECISION)
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
                    self.up_list, self.down_list = self.pc.forward(self.FORWARD_STEP)
                    self.state = self.STATE_EXPLORATION_RIGHT
            else:
                self.up_list, self.down_list = self.pc.left(self.GRID_PRECISION)
        elif self.state == self.STATE_EXPLORATION_LEFT_BACK:
            if self.multiranger.front > self.OBSTACLE_AVOIDANCE_THRESHOLD:  # no obstacle on front sensor
                # go forward and switch to right exploration
                self.up_list, self.down_list = self.pc.forward(self.FORWARD_STEP)
                self.state = self.STATE_EXPLORATION_RIGHT
            else:
                self.up_list, self.down_list = self.pc.right(self.GRID_PRECISION)
        elif self.state == self.STATE_LANDING_BACK:
            self.pc.set_default_velocity(0.05)
            self.up_list, self.down_list = self.pc.back(0.05)
            if self.up_list != []:
                self.x, self.y, self.z = self.pc.get_position()
                up_down = np.array(self.up_list) - np.array(self.down_list)
                print("up_down: ", up_down, "max - min = {}".format(max(up_down) - min(up_down)))                
                if abs(max(up_down) - min(up_down)) > 0.1 and self.x >= self.LANDING_REGION_X[0] and self.x <= self.LANDING_REGION_X[1]:
                    self.x, self.y, self.z = self.pc.get_position()
                    self.pc.back(0.2)

                    self.state = self.STATE_LANDING
        elif self.state == self.STATE_LANDING:
            #self.pc.down(self.multiranger.down)
            return False
            # end program after landing
        
        
        if self.state != self.STATE_LANDING and self.state != self.STATE_LANDING_BACK:
            
            #if any([z_value < self.DETECTION_THRESHOLD_Z for z_value in z_list]) and self.x <= self.LANDING_REGION_X[0] and self.x >= self.LANDING_REGION_X[1]:
            if self.up_list != []:
                self.x, self.y, self.z = self.pc.get_position()
                up_down = np.array(self.up_list) - np.array(self.down_list)
                print("up_down: ", up_down, "max - min = {}".format(max(up_down) - min(up_down)))                
                if abs(max(up_down) - min(up_down)) > 0.1 and self.x >= self.LANDING_REGION_X[0] and self.x <= self.LANDING_REGION_X[1]:
                    self.x, self.y, self.z = self.pc.get_position()

                    

                    if self.state == self.STATE_EXPLORATION_LEFT:
                        self.pc.go_to(self.x+0.45,self.y+0.2)
                    elif self.state == self.STATE_EXPLORATION_RIGHT:
                        self.pc.go_to(self.x+0.45,self.y-0.2)
                    self.state = self.STATE_LANDING_BACK
        """
        if self.state != self.STATE_LANDING:
            print("z_list: ", z_list)
            #if any([z_value < self.DETECTION_THRESHOLD_Z for z_value in z_list]) and self.x <= self.LANDING_REGION_X[0] and self.x >= self.LANDING_REGION_X[1]:
            if z_list != []:
                if abs(max(z_list) - min(z_list)) > 0.05 and self.x >= self.LANDING_REGION_X[0] and self.x <= self.LANDING_REGION_X[1]:
                    idx = (np.argmin(z_list)+np.argmax(z_list))/2

                    if self.state == self.STATE_EXPLORATION_LEFT:
                        edge_y = -idx / len(z_list) * self.GRID_PRECISION + self.y
                        self.pc.go_to(self.x, edge_y + 0.15)

                    elif self.state == self.STATE_EXPLORATION_RIGHT:
                        edge_y = idx/len(z_list)* self.GRID_PRECISION + self.y
                        self.pc.go_to(self.x, edge_y - 0.15)
                    self.pc.forward(0.1)
                    self.state = self.STATE_LANDING
        """
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

        with Multiranger(scf, rate_ms=50) as multiranger:
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
    #robot.TAKEOFF_REGION_X, robot.LANDING_REGION_X = robot.LANDING_REGION_X, robot.TAKEOFF_REGION_X
    robot.state= robot.STATE_EXPLORATION_RIGHT
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(0.5)
        scf.cf.param.set_value('kalman.resetEstimation','1')
        
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation','0')
        with Multiranger(scf, rate_ms=50) as multiranger:
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