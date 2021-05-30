"""
Main program of CrazyPractical, run this file to perform the project
sequence.
"""

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

import time
import cv2
import pickle
import pandas as pd
import numpy as np
from pandas.core.indexes import multi
import math

import global_navigation as nav
from custom_multiranger import CustomMultiranger
from custom_position_hl_commander import CustomPositionHlCommander


# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

# Shared constants between objects.
DEFAULT_HEIGHT = 0.3
DEFAULT_VELOCITY = 0.3
PLATFORM_SIDE = 0.25
LAND_THRESHOLD = 0.09

class Robot:
    """
    Drone class, created to handle the states of the Crazyflie and controls its movements.
    It handles map exploration, obstacle avoidance, occupancy grid generation and return
    path optimization.
    """


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


    # 0.1m = 10cm precision
    GRID_PRECISION = 0.1
    DETECTION_THRESHOLD_SIDEWAY = 8*GRID_PRECISION
    DETECTION_THRESHOLD_Z = 0.5
    OBSTACLE_AVOIDANCE_THRESHOLD = 8*GRID_PRECISION
    STEP = 6*GRID_PRECISION
    SCANNER = 8*GRID_PRECISION

    # Dimensions of takeoff and landing regions along x coordinate
    TAKEOFF_REGION_X = [0, 2]
    LANDING_REGION_X = [2, 5]

    # Dimensions of the map along y coordinate.
    Y_MIN = -0.8
    Y_MAX = 0.3

    def __init__(self, pc, multiranger, x=0, y=0, z=DEFAULT_HEIGHT):
        """
        Constructor of robot class.
        """
        ind = pd.MultiIndex.from_arrays([[]] * 2, names=('x', 'y'))
        self.dynamic_occupancy = pd.Series(
            index=ind, name='occupancy_value', dtype='int')
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
        """
        This method takes a coordinate number and truncates to be within grid precision.
        """
        return int(coordinate/self.GRID_PRECISION)*self.GRID_PRECISION

    def fill_dynamic_occupancy(self, x, y, tile_state):
        """
        This method puts a new tile state in the dynamic occupancy, knowing tile 
        coordinates x and y.
        """
        x_truncated = self.truncate_to_grid_precision(x)
        y_truncated = self.truncate_to_grid_precision(y)

        # if element exists, update with new value, else create new item
        self.dynamic_occupancy.at[x_truncated, y_truncated] = tile_state

    def update_occupancy(self):
        """
        This method checks the readings of the multiranger sensors and fills the dynamic occupancy
        with the corresponding tile states in case of obstacles.
        """
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
        """
        This method is applied on each row of the dynamic occupancy and puts 
        obstacle tiles in their corresponding position on the map.
        """
        x = row['x']
        y = row['y']
        pixel_label = row['occupancy_value']

        if pixel_label == self.TILE_OBSTACLE:
            i = int((x - self.x_min)/self.GRID_PRECISION)
            j = int((y - self.y_min)/self.GRID_PRECISION)
            self.map[i, j] = 255

    def change_dynamic_occupancy_referential(self):
        """
        This method performs a referential transformation when changing the origin from the takeoff
        position to the landing pad position and applies to the dynamic occupancy map.
        """
        old_occupancy_reset_index = self.dynamic_occupancy.reset_index()
        ind = pd.MultiIndex.from_arrays([[]] * 2, names=('x', 'y'))
        self.dynamic_occupancy = pd.Series(
            index=ind, name='occupancy_value', dtype='int')
        for idx in old_occupancy_reset_index.index:
            row = old_occupancy_reset_index.loc[idx]
            new_x = -row['x'] + self.landing_position[0]
            new_y =  row['y'] - self.landing_position[1]
            occupancy_value = row['occupancy_value']
            self.dynamic_occupancy.at[new_x,new_y] = occupancy_value
        

    def build_map(self):
        """
        This method builds a numpy array with the obstacle points previously detected
        by the dynamic occupancy mapping along the robot movement. It detects the 
        different blocks and uses pyvisgraph to generate a list of setpoints
        to follow according to A* path planning algorithm.
        """
        
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
        
        kernel = np.ones((3,3),np.uint8)
        self.map = cv2.dilate(self.map,kernel,iterations = 1)

        connected_points, _ = cv2.findContours(
            self.map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        connected_points_m = []
        for row in connected_points:
            row_m = []
            for pixel in row:
                x = pixel[0,0] * self.GRID_PRECISION + self.x_min
                y = pixel[0,1] * self.GRID_PRECISION + self.y_min
                row_m.append([x, y])
            connected_points_m.append(row_m)

        
        # build obstacles and build visibility graph.
        obstacles = nav.build_obstacles(connected_points_m)
        graph = nav.build_visgraph(obstacles)
        # apply A* to extract optimal path and target points coordinates.
        sp = nav.apply_astar(graph, [self.x, self.y], 
        [-self.takeoff_position[0] + self.landing_position[0], -self.takeoff_position[1] + self.landing_position[1] ])
        self.all_target_points = []
        for i in range(0, len(sp)):
            self.all_target_points.append([sp[i].x, sp[i].y])
        self.target_point = self.all_target_points[0]
        print(self.all_target_points)

    def find_next_target_point(self):
        """
        find the next target points in the list of all target points
        """
        i = self.all_target_points.index(self.target_point)
        self.target_point[0] = self.all_target_points[i+1][0]
        self.target_point[1] = self.all_target_points[i+1][1]

    def behave_return(self):
        """
        This method handles the movement of the robot when moving back to the takeoff pad.
        It moves each time to a target setpoint. If at any time, an obstacle is detected, it recreates the
        map and generates new setpoints.
        """
        self.x, self.y, self.z = self.pc.get_position()
        self.GRID_PRECISION = 0.1
        self.STEP = 0.1
        if self.truncate_to_grid_precision(self.x) == self.truncate_to_grid_precision(self.all_target_points[-1][0]) and \
                self.truncate_to_grid_precision(self.y) == self.truncate_to_grid_precision(self.all_target_points[-1][1]):
            return False
        elif self.multiranger.front < self.GRID_PRECISION or self.multiranger.back < self.GRID_PRECISION or \
                self.multiranger.left < self.GRID_PRECISION or self.multiranger.right < self.GRID_PRECISION:
            self.build_map()
            self.find_next_target_point()
        if self.truncate_to_grid_precision(self.x) == self.truncate_to_grid_precision(self.target_point[0]) and \
                self.truncate_to_grid_precision(self.y) == self.truncate_to_grid_precision(self.target_point[1]):
            self.find_next_target_point()
        self.pc.go_to(self.target_point[0], self.target_point[1])

        return True

    def detect_edge(self):
        """
        This method detects a brutal variation in z state estimation after each step and returns 
        its position relative to the drone's position before the step. Returns none if no z variation
        detected.
        """
        if self.up_list != []:
            self.x, self.y, self.z = self.pc.get_position()
            up_down = np.array(self.up_list) - np.array(self.down_list)
            print("up_down: ", up_down, "max - min = {}".format(max(up_down) - min(up_down)))
            if abs(max(up_down) - min(up_down)) > LAND_THRESHOLD and self.x >= self.LANDING_REGION_X[0] and self.x <= self.LANDING_REGION_X[1]:
                idx = (np.argmin(up_down) + np.argmax(up_down)) / 2
                distance_start_edge = idx / len(up_down) * self.STEP
                print("Distance to edge is", distance_start_edge)
                return distance_start_edge
            else:
                return None
        else:
            return None

    def detect_obstacles_while_moving(self):
        """
        #### UNUSED ####
        This method is called after each step. Tables of values of TOF sensors during the step
        are checked. If some are below the SCANNER value, the map's corresponding tiles are
        updated as obstacle tiles.
        """
        if self.state == self.STATE_EXPLORATION_RIGHT:
            if any(np.array(self.front_list < self.SCANNER)):

                # Get ids in list where obstacle near
                idx_list = np.where(np.array(self.front_list) < self.SCANNER)
                # convert ids into y coordinate
                y_obstacles = self.y_before_step - idx_list[0]*9/100*self.STEP
                scan = np.array(self.front_list)
                # calculate x coordinate of obstacles basted on sensor value
                x_obstacles = self.x + scan[idx_list[0]]

                return y_obstacles, x_obstacles

        elif self.state == self.STATE_EXPLORATION_LEFT:
            if any(np.array(self.front_list < self.SCANNER)):

                idx_list = np.where(np.array(self.front_list) < self.SCANNER)
                y_obstacles = self.y_before_step + idx_list[0]*9/100*self.STEP
                scan = np.array(self.front_list)
                x_obstacles = self.x + scan[idx_list[0]]

                return y_obstacles, x_obstacles

        elif self.state == self.STATE_FORWARD:
            if any(np.array(self.left_list < self.SCANNER)):

                idx_list = np.where(np.array(self.front_list) < self.SCANNER)
                x_obstacles = self.x_before_step + idx_list[0] * 9 / 100 * self.STEP # step when going forward
                scan = np.array(self.front_list)
                y_obstacles = self.x + scan[idx_list[0]]

                return y_obstacles, x_obstacles

            if any(np.array(self.right_list < self.SCANNER)):

                idx_list = np.where(np.array(self.front_list) < self.SCANNER)
                x_obstacles = self.x_before_step + idx_list[0] * 9 / 100 * self.STEP # step when going forward
                scan = np.array(self.front_list)
                y_obstacles = self.x - scan[idx_list[0]]

                return y_obstacles, x_obstacles
        else:
            pass

    def behave_explore(self):
        """
        This method is used to handle the states and movements of the robot during the exploration of the map.
        It is called periodically at each iteration and always returns True until the landing pad is properly
        detected and the robot is ready to land.
        """
        # storing the robot's position before movement
        if self.state != self.STATE_FORWARD_LAND and self.state != self.STATE_LEFT_LAND and self.state != self.STATE_RIGHT_LAND:
            self.x_before_step, self.y_before_step, self.z_before_step = self.pc.get_position()

        if self.x >= self.LANDING_REGION_X[0] and self.x <= self.LANDING_REGION_X[1]:
            self.DETECTION_THRESHOLD_SIDEWAY = 4 * self.GRID_PRECISION
            self.OBSTACLE_AVOIDANCE_THRESHOLD =  4* self.GRID_PRECISION
            self.STEP = 2* self.GRID_PRECISION
        else:
            self.DETECTION_THRESHOLD_SIDEWAY = 8 * self.GRID_PRECISION
            self.OBSTACLE_AVOIDANCE_THRESHOLD = 8 * self.GRID_PRECISION
            self.STEP = 6 * self.GRID_PRECISION
        
        self.x, self.y, self.z = self.pc.get_position()
        print("state: ", self.state, " position: {0} {1} {2}".format(self.x,self.y,self.z), " left: {0}, right {1}".format(self.multiranger.left, self.multiranger.right))
        self.down_list = []
        self.up_list = []
        # Handling state when the robot is the exploring to the right.
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
                    self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.forward(self.STEP)
                    self.edge_displacement = self.detect_edge()
                    if self.edge_displacement is not None:
                        self.state = self.STATE_FORWARD_LAND
                    else:
                        self.state = self.STATE_EXPLORATION_LEFT

            else:
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.right(self.STEP)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_RIGHT_LAND

        # Handling state when the robot is backing up to the left after detecting an obstacle from the front sensor.
        elif self.state == self.STATE_EXPLORATION_RIGHT_BACK:
            if self.multiranger.front > self.OBSTACLE_AVOIDANCE_THRESHOLD:  # no obstacle on front sensor
                # go forward and switch to left exploration
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.forward(self.STEP)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_FORWARD_LAND
                else:
                    self.state = self.STATE_EXPLORATION_LEFT
                self.state = self.STATE_EXPLORATION_LEFT
            else:
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.left(self.STEP)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_LEFT_LAND
        # Handling state when the robot is the exploring to the left.
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
                    self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.forward(self.STEP)
                    self.edge_displacement = self.detect_edge()
                    if self.edge_displacement is not None:
                        self.state = self.STATE_FORWARD_LAND
                    else:
                        self.state = self.STATE_EXPLORATION_RIGHT
            else:
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.left(self.STEP)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_LEFT_LAND
        # Handling state when the robot is backing up to the right after detecting an obstacle from the front sensor.
        elif self.state == self.STATE_EXPLORATION_LEFT_BACK:
            if self.multiranger.front > self.OBSTACLE_AVOIDANCE_THRESHOLD:  # no obstacle on front sensor
                # go forward and switch to right exploration
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.forward(self.STEP)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_FORWARD_LAND
                else:
                    self.state = self.STATE_EXPLORATION_RIGHT
            else:
                self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.right(self.STEP)
                self.edge_displacement = self.detect_edge()
                if self.edge_displacement is not None:
                    self.state = self.STATE_RIGHT_LAND
        # Handling state when the robot detected the landing pad while moving forward.
        elif self.state == self.STATE_FORWARD_LAND:
            self.x, self.y, self.z = self.pc.get_position()
            self.pc.go_to(self.x_before_step + self.edge_displacement + PLATFORM_SIDE/2, self.y + 0.5)
            self.state = self.LANDING_MANEUVER_RIGHT
            self.edge_displacement = None
        # Handling state when the robot detected the landing pad while moving right.
        elif self.state == self.STATE_RIGHT_LAND:
            self.x, self.y, self.z = self.pc.get_position()
            self.pc.go_to(self.x + 0.5, self.y_before_step - self.edge_displacement - PLATFORM_SIDE/2)
            self.state = self.LANDING_MANEUVER_BACK
            self.edge_displacement = None
        # Handling state when the robot detected the landing pad while moving left.
        elif self.state == self.STATE_LEFT_LAND:
            self.x, self.y, self.z = self.pc.get_position()
            self.pc.go_to(self.x + 0.5, self.y_before_step + self.edge_displacement + PLATFORM_SIDE/2)
            self.state = self.LANDING_MANEUVER_BACK
            self.edge_displacement = None
        # Handling state when the robot is backing up, looking for the landing pad after having detected an edge while approaching
        # from the right or left
        elif self.state == self.LANDING_MANEUVER_BACK:
            self.pc.set_default_velocity(0.1)
            self.STEP = 0.15
            self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.back(self.STEP)
            LAND_THRESHOLD = 0.065
            self.edge_displacement = self.detect_edge()
            LAND_THRESHOLD = 0.09
            if self.edge_displacement is not None:
                self.x, self.y, self.z = self.pc.get_position()
                self.pc.go_to(self.x_before_step - self.edge_displacement - PLATFORM_SIDE/2, self.y_before_step )
                self.state = self.STATE_LANDING
                self.edge_displacement = None
        # Handling state when the robot is backing up from the right, looking for the landing pad after having detected an edge while approaching
        # from the front
        elif self.state == self.LANDING_MANEUVER_RIGHT:
            self.pc.set_default_velocity(0.1)
            self.STEP = 0.15
            self.up_list, self.down_list, self.front_list, self.back_list, self.left_list, self.right_list = self.pc.right(self.STEP)
            LAND_THRESHOLD = 0.065
            self.edge_displacement = self.detect_edge()
            LAND_THRESHOLD = 0.09
            if self.edge_displacement is not None:
                self.x, self.y, self.z = self.pc.get_position()
                self.pc.go_to(self.x_before_step, self.y_before_step - self.edge_displacement - PLATFORM_SIDE/2 )
                self.state = self.STATE_LANDING
                self.edge_displacement = None
        # Handling state of landing, storing landing coordinates
        elif self.state == self.STATE_LANDING:
            self.x, self.y, self.z = self.pc.get_position()
            self.landing_position = [self.x, self.y]
            return False
        
        self.x, self.y, self.z = self.pc.get_position()
        # always return true before the landing
        return True


def main_sequence():
    """
    Main sequence of robot movement. It handles takeoff, map exploration, obstacle avoidance,
    landing pad detection, landing, then building of an occupancy grid, returning to start position
    and finally landing.
    """
    robot = Robot(None, None, 0, 0, 0)

    # Forward run
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(0.5)
        scf.cf.param.set_value('kalman.resetEstimation','1')        
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation','0')
        time.sleep(0.1)
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
                
            landing_yaw = multiranger.state_estimate_yaw
    # build occupancy map            
    time.sleep(2)
    robot.state = robot.STATE_EXPLORATION_RIGHT
    robot.x, robot.y = 0,0
    robot.change_dynamic_occupancy_referential()
    robot.build_map()
    # return run.
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(0.5)
        scf.cf.param.set_value('kalman.resetEstimation','1')        
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation','0')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation','1')        
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation','0')

        scf.cf.param.set_value('kalman.initialX',str(robot.x))
        scf.cf.param.set_value('kalman.initialY',str(robot.y))
        scf.cf.param.set_value('kalman.initialYaw',str(landing_yaw - math.pi))
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
                while robot.behave_return() == True:
                    robot.update_occupancy()
                # To compensate for estimation drift, start looking again for takeoff pad
                robot.pc.back(2*robot.GRID_PRECISION)
                while robot.behave_explore() == True:
                    robot.update_occupancy()


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    main_sequence()
