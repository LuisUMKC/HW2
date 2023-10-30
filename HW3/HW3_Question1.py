# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 22:25:18 2023

@author: mario
"""

from HW2_Question2 import Obstacle
from Astarpath import AStartAlgorithm

if __name__ == "__main__":
    obs_pos = [(1, 1), (4, 4), (3, 4), (5, 0), (5, 1),
               (0, 7), (1, 7), (2, 7), (3, 7)]
    obs_radius = 0.25
    obs_list = [Obstacle(each_ob[0], each_ob[1], obs_radius)
                for each_ob in obs_pos]
    astart = AStartAlgorithm(0, 0, 10, 10, 0.5)
    route = astart.run(start=(0, 0), goal=(8, 9), r_radius=0, obs_list=obs_list)
    astart.plot_route(route=route)