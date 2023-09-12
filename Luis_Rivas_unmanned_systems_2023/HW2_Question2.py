# -*- coding: utf-8 -*-
"""
Created on Mon Sep 11 11:27:45 2023

@author: mario
"""

import numpy as np 
import matplotlib.pyplot as plt 


class Obstacle():
    def __init__(self, x_pos:float, y_pos:float, radius:float):
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius
        
    def is_inside(self, curr_x:float, curr_y:float,
                  robot_radius:float=0) -> bool:
        dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
        
        if dist_from > self.radius + robot_radius:
            return False
        
        return True
def is_not_valid(obs_list:list, x_min:int, y_min:int, x_max:int, 
                 y_max:int, x_curr:float, y_curr:float, agent_radius:float=0.0):
    
    for obs in obstacle_list:
        if obs.is_inside(x_curr, y_curr,agent_radius):
            print("You're dead at ", obs.x_pos, obs.y_pos)
            if obs.is_outside(agent_x, agent_y, agent_radius):
                print("You're safe at ", agent_x, agent_y)
            return True
    
    if x_min > x_curr:
        return True
    if x_max < x_curr:
        return True
    if y_min > y_curr:
        return True
    if y_max < y_curr:
        return True
    
    return False

if __name__=="__main__":

    obstacle_position =  [(1,1), (4,4), (3,4), (5,0), (5,1), (0,7), (1,7), (2,7), (3,7)]
    obstacle_list = [] 
    agent_radius = 0.25 #Given D = 0.5, also step size
    
    for obs_pos in obstacle_position:
        obstacle = Obstacle(obs_pos[0], obs_pos[1], agent_radius)
        obstacle_list.append(obstacle) 
        
    
    agent_x = 2
    agent_y = 2
    
    
    for obs in obstacle_list:
        print("This obstacles position is", obs.x_pos, obs.y_pos)
        if obs.is_inside(agent_x, agent_y,agent_radius):
            print("You're dead at ", obs.x_pos, obs.y_pos)
            break


    fig, ax = plt.subplots() 
    ax.set_xlim(-1, 10)
    ax.set_ylim(-1, 10)    
    for obs in obstacle_list:
        obs_plot = plt.Circle((obs.x_pos, obs.y_pos), obs.radius, color='blue')
        ax.add_patch(obs_plot)
    
    agent_plot = plt.Circle((agent_x, agent_y), agent_radius, color='red')
    ax.add_patch(agent_plot)
    plt.show()