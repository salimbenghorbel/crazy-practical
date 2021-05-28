# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
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
This script shows the basic use of the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

The MotionCommander uses velocity setpoints.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
import numpy as np
import matplotlib.pyplot as plt

URI = 'radio://0/80/2M/E7E7E7E7E7'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

DEFAULT_HEIGHT = 0.3
PLATFORM_HEIGHT = 0.1
PLATFORM_SIDE = 0.3
ARENA_WIDTH = 3
ARENA_LENGTH = 5
REGIONS_LENGTH = 1.5
MAP_RESOLUTION = 0.1
'''
class Map:
    def __init__(self, ARENA_WIDTH, ARENA_LENGTH, REGIONS_LENGTH, PLATFORM_SIDE, MAP_RESOLUTION):
        self.table = np.zeros()
'''
class Robot:
    # x+ is front
    # x- is back
    # y+ is left
    # y- is right
    def __init__(self, commander, x, y, z, multiranger):
        self.mc = commander
        self.x = x
        self.y = y
        self.z = z
        self.multi = multiranger

    def forward(self, dx):
        self.mc.forward(dx)
        self.x += dx

    def back(self, dx):
        self.mc.back(dx)
        self.x -= dx

    def right(self, dy):
        self.mc.right(dy)
        self.y -= dy

    def left(self, dy):
        self.mc.left(dy)
        self.y += dy

    def up(self, dz):
        self.mc.up(dz)
        self.z += dz

    def down(self, dz):
        self.mc.down(dz)
        self.z -= dz

    def start_forward(self, vx):
        self.mc.start_forward(vx)
        return time.time()

    def start_back(self, vx):
        self.mc.start_back(vx)
        return time.time()

    def start_left(self, vy):
        self.mc.start_left(vy)
        return time.time()

    def start_right(self, vy):
        self.mc.start_right(vy)
        return time.time()

    def go_forward_til_obstacle(self, vx):
        start = self.start_forward(vx)
        while self.multi.front > 0.2:
            time.sleep(0.05)
            self.x += vx*0.05
        self.mc.stop()
        dt = time.time() - start
        self.x += dt * vx

    def detect_edge(self, z_threshold):
        old_z_ranger = self.multi.down
        while abs(self.multi.down - old_z_ranger) < z_threshold:
            # print('z ranger was', old_z_ranger, 'z ranger is now', self.multi.down)
            old_z_ranger = self.multi.down
            # time.sleep(0.01)
        self.mc.stop()

    def detect_arena(self, v):

        print('detecting arena')

        time.sleep(1.5)
        start = self.start_left(v)
        self.detect_edge(0.008)
        dt = time.time() - start
        self.y += dt * v
        y_left_edge = self.y
        self.mc.stop()
        self.right(0.1)

        time.sleep(5)

        start = self.start_back(v)
        self.detect_edge(0.008)
        dt = time.time() - start
        time.sleep(1.5)
        self.x -= dt * v
        x_front_edge = self.x
        self.mc.stop()
        self.forward(0.1)

    def z_scan_left(self, vy):

        z_list = []
        top_list = []
        y_list = []
        start = self.start_left(vy)

        while (time.time() - start)*vy < 0.8:
            z_list.append(self.multi.down)
            top_list.append(self.multi.up)
            y_list.append(self.y + (time.time() - start)*vy)
            time.sleep(0.05)
        self.mc.stop()

        plt.plot(y_list, z_list, 'r')
        plt.plot(y_list, top_list, 'b')
        plt.plot(y_list, [top_list_i - z_list_i for top_list_i, z_list_i in zip(top_list, z_list)], 'g')
        plt.show()
        return y_list, z_list

    def detect_platform(self, v):

        print('detecting platform')

        start = self.start_left(v)
        self.detect_edge(0.03)
        dt = time.time() - start
        self.y += dt * v
        y_left_edge = self.y
        self.right(dt * v)

        start = self.start_forward(v)
        self.detect_edge(0.02)
        dt = time.time() - start
        self.x += dt * v
        x_front_edge = self.x
        self.back(dt * v)
        return (x_front_edge - PLATFORM_SIDE), x_front_edge, (y_left_edge - PLATFORM_SIDE), y_left_edge

    def go_to_landing(self, v):
        start = self.start_forward(v)
        self.detect_edge(0.03)
        print('front edge detected')
        dt = time.time() - start
        self.x += dt * v
        x_front_edge = self.x
        self.mc.forward(PLATFORM_SIDE/2)

        start = self.start_left(v)
        self.detect_edge(0.025)
        print('left edge detected')
        dt = time.time() - start
        self.y += dt * v
        y_left_edge = self.y
        self.mc.right(PLATFORM_SIDE / 2)

#    def search_landing_region(self, v):





if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with Multiranger(scf, rate_ms=100) as mtr:
            with MotionCommander(scf, DEFAULT_HEIGHT) as mc:
                time.sleep(1)

                craycray = Robot(mc, 0, 0, 0, mtr)
                # There is a set of functions that move a specific distance
                # We can move in all directions
                #craycray.go_forward_til_obstacle(0.3)
                # craycray.go_to_landing(0.3)
                craycray.z_scan_left(0.2)
                time.sleep(2)
                craycray.mc.land()
