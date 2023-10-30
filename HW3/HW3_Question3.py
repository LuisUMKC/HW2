# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 22:26:56 2023

@author: mario
"""

from random_path import RRT
from HW2_Question2 import Obstacle
import matplotlib.pyplot as plt

if __name__ == "__main__":
    obs_pos = [(2,2), (2,3), (2,4), (5,5), (5,6), (6,6), (7,3), (7,4), (7,5), (7,6), (8,6)]
    obs_radius = 0.25
    obs_list = [Obstacle(each_ob[0], each_ob[1], obs_radius)
                for each_ob in obs_pos]
    
    rrt = RRT(
        minx=0,
        maxx=10,
        miny=0,
        maxy=10,
        gs=0.5,
        obs_list=obs_list,
        iterations=10000
    )
    path, tree = rrt.run(start=(1,1), goal=(8, 9))
    plt.figure(figsize=(8, 6))
    plt.plot([node.x for node in tree], [node.y for node in tree], 'bo', markersize=0.25 * 10)

    for obs in obs_list:
        circle = plt.Circle((obs.x, obs.y), obs_radius, color='black')
        plt.gca().add_patch(circle)

    plt.plot([point[0] for point in path], [point[1] for point in path], 'r')

    # Draw start and goal nodes
    plt.plot(1, 1, 'ro', markersize=0.25 * 10)
    plt.plot(8, 9, 'go', markersize=0.25 * 10)

    plt.xlim(-1, 11)
    plt.ylim(-1, 11)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.show()