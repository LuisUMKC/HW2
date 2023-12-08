# -*- coding: utf-8 -*-
"""
Created on Thu Dec  7 15:12:27 2023

@author: mario
"""

from itertools import permutations
import time
import tqdm

from AStarAlgorithm import AStarAlgorithm
from Dijkstras import Obstacle
import matplotlib.pyplot as plt
import numpy as np

obs_radius = 0.265
robot_radius = 0.6  
x_min = 0
x_max = 15
y_min = 0
y_max = 15
grid_space = 0.5

waypoints = [(1,1), (9,7), (1,9), (4,4), (9,4), (6,14), (3,11), (14,1), (1,14), (14,14), (7,10)]

ox = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8, 8, 8, 8, 2, 3, 4, 5, 6, 9, 10, 11, 12, 15, 2, 2, 2,  2,  2,  2,  5, 5,  5,  5,  5,  6,  7,  8,  9,  10, 11, 12, 12, 12, 12, 12]
oy = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2,  2,  2,  2,  3, 4, 5, 6, 7, 8, 9, 7, 7, 7, 7, 7, 6, 6,  6,  6,  6,  8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13, 12, 12, 12, 12, 12, 12, 8,  9,  10, 11, 12]

obs_list = [Obstacle(ox[i], oy[i], obs_radius)
            for i in range(len(ox))]

other_waypoints = {}
cost_dictionary = {}
paths_dictionary = {}

#Compute all the costs from some waypoint to all other waypoints
compute_times = 0
for wp in waypoints: 
    for other_wp in waypoints:
        if wp == other_wp:
            continue
        if (wp, other_wp) in cost_dictionary:
            continue
        if (other_wp, wp) in cost_dictionary:
            continue
        else:
            compute_times += 1
            ## this would be where you plug in astar
            #waypoints = call_astar
            #total distance = sum of all the distances in the path
            # total_distance = m.dist(wp, other_wp)
            astart = AStarAlgorithm(0, 0, 15, 15, 0.5)
            route, gcost, hcost, cost = astart.run(start=(wp[0], wp[1]), goal=(other_wp[0], other_wp[1]), r_radius=0.5, obs_list=obs_list)
            total_distance = gcost
            #a->c
            cost_dictionary[wp, other_wp] = total_distance
            #c->a 
            cost_dictionary[other_wp, wp] = total_distance
            paths_dictionary[other_wp, wp] = [[each[0], each[1]] for each in route]
            paths_dictionary[wp, other_wp] = [[each[0], each[1]] for each in route]

# Compute all the possible paths
paths = list(permutations(waypoints, len(waypoints)))
total_cost = []
print("Number of times computed: ", compute_times)
print("Number of paths found: ", len(paths))


n_iterations = 100

start_time = time.time()
for j, path in tqdm.tqdm(enumerate(paths)):    
    sum_cost = 0
    if j % n_iterations == 0:
        pass
        # print("Iteration: ", j)
    
    for i in range(len(path)-1):
        sum_cost += cost_dictionary[path[i], path[i+1]]
        
    total_cost.append(sum_cost)
        
end_time = time.time()
print("Time taken", end_time - start_time)
# get best path
min_total_cost = min(total_cost)
min_total_cost_index = total_cost.index(min_total_cost)
best_path = paths[min_total_cost_index]
print("best path", best_path)
print("min cost", min_total_cost)

final_path = []

# for i in range(0, len(best_path)-1):
#     final_path.extend(paths_dictionary[(best_path[i], best_path[i+1])])

fig, ax = plt.subplots()

x_values = np.arange(x_min, x_max + grid_space, grid_space)
y_values = np.arange(y_min, y_max + grid_space, grid_space)

# for y, x in zip(y_values, x_values):
#     index = compute_index(x_min, x_max, y_min, y_max, grid_space, x, y)
    # plt.text(x, y, str(index), color='red', fontsize=8, ha='center', va='center')

for obs in obs_list:
    obs_plot = plt.Circle((obs.x, obs.y), obs.radius, color='black')
    ax.add_patch(obs_plot)

for p in waypoints:
    obs_plot = plt.Circle((p[0], p[1]), 0.2, color='red')
    ax.add_patch(obs_plot)

# obs_plot = plt.Circle((self.goal[0], self.goal[1]), self.r_radius, color='green')
# ax.add_patch(obs_plot)
for i in range(0, len(best_path)-1):
    p = paths_dictionary[(best_path[i], best_path[i+1])]
    plt.plot([x[0] for x in p], [x[1] for x in p], c='blue')

plt.xlim(x_min - grid_space, x_max + grid_space)
plt.ylim(y_min - grid_space, y_max + grid_space)
plt.show()