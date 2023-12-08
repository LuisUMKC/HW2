# -*- coding: utf-8 -*-
"""
Created on Thu Dec  6 10:20:55 2023

@author: mario
"""

#!/usr/bin/env python3
from Dijkstras import Obstacle, compute_index
from AStarAlgorithm import AStarAlgorithm
import itertools
import numpy as np
import matplotlib.pyplot as plt




obs_radius = 0.1
robot_radius = 0.5 
x_min = 0
y_min = 0
x_max = 15
y_max = 15
grid_space = 0.5


ox = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 
      8, 8, 8, 8, 8, 8, 8, 2, 3, 4, 5, 6, 9, 10, 11, 12, 15, 2, 2, 2,  
      2,  2,  2,  5, 5,  5,  5,  5,  5,  5,  6,  7,  8,  9,  10, 11, 12, 
      12, 12, 12, 12]
oy = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 
      2,  2,  2,  2,  3, 4, 5, 6, 7, 8, 9, 7, 7, 7, 7,
      7, 6, 6,  6,  6,  6,  8, 9, 10, 11, 12, 13, 9, 10,
      11, 12, 13, 14, 15, 12, 12, 12, 12, 12, 12, 8,  9, 
      10, 11, 12]


obs_list = [Obstacle(ox[i], oy[i], obs_radius)
            for i in range(len(ox))]

waypoints = [(0, 0), (9,4), (4,4), (1,9), (9,9), (6,14)]  


distance_table = {}
paths_table = {}

for coords1 in waypoints:
    for coords2 in waypoints:
        if coords1 != coords2 and (distance_table.get((coords1, coords2)) == None and distance_table.get((coords2, coords1)) == None) :
            astart = AStarAlgorithm(0, 0, 15, 15, 0.5)
            route, gcost, hcost, cost = astart.run(start=(coords1[0], coords1[1]), goal=(coords2[0], coords2[1]), r_radius=0.5, obs_list=obs_list)
            paths_table[(coords1, coords2)] = [[each[0], each[1]] for each in route]
            paths_table[(coords2, coords1)] = [[each[0], each[1]] for each in route]
            print("path taken",coords1, coords2, round(gcost), round(hcost),round(cost))
            distance_table[(coords1, coords2)] = cost
best_path = None
min_distance = float('inf')

waypoint_permutations = list(itertools.permutations(waypoints, len(waypoints)))

for perm in waypoint_permutations:
    if perm[0] != (0, 0):
        continue
    current_distance = 0
    for i in range(len(perm) - 1):
        if distance_table.get((perm[i], perm[i+1]), None) is not None:
            current_distance += distance_table.get((perm[i], perm[i+1]))
        else:
            current_distance += distance_table.get((perm[i+1], perm[i]))

    if current_distance < min_distance:
        min_distance = current_distance
        best_path = perm


final_path = []

for i in range(0, len(best_path)-1):
    final_path.extend(paths_table[(best_path[i], best_path[i+1])])


fig, ax = plt.subplots()

x_values = np.arange(x_min, x_max + grid_space, grid_space)
y_values = np.arange(y_min, y_max + grid_space, grid_space)

for y, x in zip(y_values, x_values):
    index = compute_index(x_min, x_max, y_min, y_max, grid_space, x, y)
    

for obs in obs_list:
    obs_plot = plt.Circle((obs.x, obs.y), obs.radius, color='black')
    ax.add_patch(obs_plot)

for p in waypoints:
    obs_plot = plt.Circle((p[0], p[1]), 0.2, color='red')
    ax.add_patch(obs_plot)


ax.add_patch(obs_plot)
plt.plot([x[0] for x in final_path], [x[1] for x in final_path], c='blue')
plt.xlim(x_min - grid_space, x_max + grid_space)
plt.ylim(y_min - grid_space, y_max + grid_space)
plt.show()

