# -*- coding: utf-8 -*-
"""
Created on Mon Sep 11 14:35:14 2023

@author: mario
"""

#% Class
class Node:
    
    
    def __init__(self,x,y,parent_cost,index):
        self.x = x
        self.y = y
        self.parent_cost = parent_cost 
        self.index = index 

def compute_index(min_x, max_x, min_y, max_y, grid_spacing, cx, cy) -> int:

    index = ((cx - min_x)/grid_spacing) + \
            (((cy - min_y)/grid_spacing) * \
            (((max_x + grid_spacing)-min_x)/grid_spacing))

    return int(index)