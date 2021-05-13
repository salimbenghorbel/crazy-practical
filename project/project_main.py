# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
This script shows the basic use of the PositionHlCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system.

The PositionHlCommander uses position setpoints.

Change the URI variable to your Crazyflie configuration.
"""
import cflib.crtp
import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
import pandas as pd
from pandas.core.indexes import multi

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
    STATE_EXPLORATION = 0
    STATE_OBSTACLE_AVOIDANCE_LEFT = 1
    STATE_OBSTACLE_AVOIDANCE_BACK = 2
    STATE_OBSTACLE_AVOIDANCE_RIGHT = 3
    STATE_OBSTACLE_AVOIDANCE_FRONT = 4
    STATE_LANDING = 5
    
    # 0.1m = 10cm precision
    GRID_PRECISION = 0.1
    DETECTION_THRESHOLD_SIDEWAY = 0.5
    DETECTION_THRESHOLD_Z = 0.3

    TAKEOFF_REGION_X = [0,2]
    LANDING_REGION_X = [3,6]


    def __init__(self, pc, multiranger, x=0, y=0, z=DEFAULT_HEIGHT):
        ind = pd.MultiIndex.from_arrays([[]] * 2, names=('x','y'))
        self.dynamic_occupancy = pd.Series(index=ind ,name='occupancy_value',dtype='float64')
        self.x = x
        self.y = y
        self.z = z

        self.x_before_obstacle_avoidance = None
        self.y_before_obstacle_avoidance = None

        self.fill_dynamic_occupancy(self.x, self.y, self.TILE_TAKEOFF)
        self.state = self.STATE_EXPLORATION

    def truncate_to_grid_precision(self, coordinate):
        return int(coordinate/self.GRID_PRECISION)*self.GRID_PRECISION

    def fill_dynamic_occupancy(self, x, y, tile_state):
        x_truncated = self.truncate_to_grid_precision(x)
        y_truncated = self.truncate_to_grid_precision(y)
        
        # if element exists, update with new value, else create new item
        self.dynamic_occupancy.at[x_truncated,y_truncated] = tile_state

    def update_occupancy(self):
        self.x, self.y, self.z = self.pc.get_position()
        
        if self.multiranger.front < self.DETECTION_THRESHOLD_SIDEWAY:
            self.update_occupancy(self.x+self.multiranger.front,self.y, self.TILE_OBSTACLE)

        if self.multiranger.left < self.DETECTION_THRESHOLD_SIDEWAY:
            self.update_occupancy(self.x,self.y+self.multiranger.left, self.TILE_OBSTACLE)    

        if self.multiranger.back < self.DETECTION_THRESHOLD_SIDEWAY:
            self.update_occupancy(self.x-self.multiranger.back,self.y, self.TILE_OBSTACLE)

        if self.multiranger.right < self.DETECTION_THRESHOLD_SIDEWAY:
            self.update_occupancy(self.x,self.y-self.multiranger.right, self.TILE_OBSTACLE)

        if self.multiranger.down < self.DETECTION_THRESHOLD_Z:
            if self.x <= self.TAKEOFF_REGION_X[0] and self.x >= self.TAKEOFF_REGION_X[1]:
                self.update_occupancy(self.x, self.y, self.TILE_TAKEOFF)
            elif self.x <= self.LANDING_REGION_X[0] and self.x >= self.LANDING_REGION_X[1]:
                self.update_occupancy(self.x, self.y, self.TILE_LANDING)
        else:
            self.update_occupancy(self.x, self.y, self.TILE_FREE)

    def behave(self):
        self.x, self.y, self.z = self.pc.get_position()
        if self.state == self.STATE_EXPLORATION:
            if self.multiranger.front < self.GRID_PRECISION: # entering obstacle avoidance, going left
                self.x_before_obstacle_avoidance, self.y_before_obstacle_avoidance = self.x, self.y
                self.state = self.STATE_OBSTACLE_AVOIDANCE_LEFT
            elif self.multiranger.down < self.DETECTION_THRESHOLD_Z: 
                if self.x <= self.LANDING_REGION_X[0] and self.x >= self.LANDING_REGION_X[1]:
                    self.state = self.STATE_LANDING            
            else:
                self.pc.forward(self.GRID_PRECISION)

        elif self.state == self.STATE_OBSTACLE_AVOIDANCE_LEFT:
            if self.multiranger.front > self.GRID_PRECISION: # no obstacle on front sensor, going forward
                self.state = self.STATE_OBSTACLE_AVOIDANCE_FRONT
            elif self.multiranger.left < self.GRID_PRECISION: # obstacle on left sensor, going back
                self.state = self.STATE_OBSTACLE_AVOIDANCE_BACK
            else:
                self.pc.left(self.GRID_PRECISION)

        elif self.state == self.STATE_OBSTACLE_AVOIDANCE_BACK:
            if self.multiranger.left > self.GRID_PRECISION: # no obstacle on left sensor, going left
                self.state = self.STATE_OBSTACLE_AVOIDANCE_LEFT
            elif self.multiranger.back < self.GRID_PRECISION: # obstacle on back sensor, going right
                self.state = self.STATE_OBSTACLE_AVOIDANCE_RIGHT
            else:
                self.pc.back(self.GRID_PRECISION)

        elif self.state == self.STATE_OBSTACLE_AVOIDANCE_RIGHT:
            if self.multiranger.back > self.GRID_PRECISION: # no obstacle on back sensor, going back
                self.state = self.STATE_OBSTACLE_AVOIDANCE_BACK
            elif self.multiranger.right < self.GRID_PRECISION: # obstacle on right sensor, going forward
                self.state = self.STATE_OBSTACLE_AVOIDANCE_FRONT
            else:
                self.pc.right(self.GRID_PRECISION)

        elif self.state == self.STATE_OBSTACLE_AVOIDANCE_FRONT:
            #if back on same line as before entering obstacle avoidance, switch to explore
            if self.truncate_to_grid_precision(self.y) == self.truncate_to_grid_precision(self.y_before_obstacle_avoidance):
                self.state = self.STATE_EXPLORATION
            elif self.multiranger.right > self.GRID_PRECISION: # no obstacle on right sensor, going right
                self.state = self.STATE_OBSTACLE_AVOIDANCE_RIGHT
            elif self.multiranger.front < self.GRID_PRECISION: # obstacle on front sensor, going left
                self.state = self.STATE_OBSTACLE_AVOIDANCE_LEFT
            else:
                self.pc.forward(self.GRID_PRECISION)

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
                
                #calibrate yaw
                pc._hl_commander.go_to(0,0,0.5,1.57,2)
                time.sleep(2)
                pc._hl_commander.go_to(0, 0, 0.5, 0, 2)
                time.sleep(2)
                
                
                x,y,z = pc.get_position()
                robot = Robot(pc, multiranger, x,y,z)

                while robot.behave() == True:
                    robot.update_occupancy()
                    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    main_sequence()
