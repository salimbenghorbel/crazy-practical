"""
This module contains different method to perform global navigation of the drone.
It builds obstacle objects and generates visibility graph and finally applies
A* algorithm to get optimal path to be followed.
The method use pyvisgraph and are based off the examples from:
    https://github.com/TaipanRex/pyvisgraph
"""

import pyvisgraph as vg

def build_obstacles(vertices_array, drone_clearance_m):
    """
    This method takes as input an array of vertices, map dimensions and drone 
    clearance to build obstacle object list which will be used with pyvisgraph API.
    """
    obstacles = []
        
    for i in range(0,len(vertices_array)):
        polygon = []
        for j in range(0,len(vertices_array[i])):
            x = vertices_array[i][j][0]
            y = vertices_array[i][j][1]
            polygon.append(vg.Point(x, y ))
        obstacles.append(polygon)
    return obstacles

def build_visgraph(obstacles):
    """
    This method uses pyvisgraph API to build a visibility graph which will be 
    used by the A* algorithm.
    """
    graph = vg.VisGraph()
    graph.build(obstacles)
    return graph

def apply_astar(graph, start, goal):
    """
    This method applies the A* algorithm to find shortest path to be followed
    by drone.
    """
    start_point = vg.Point(start[0],start[1])
    goal_point = vg.Point(goal[0],goal[1])
    return graph.shortest_path(start_point,goal_point)