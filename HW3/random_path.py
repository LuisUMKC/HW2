# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 22:17:02 2023

@author: mario
"""


from HW1_Node import Node 
from HW2_Q1 import calculate_distance
import random
import math

class RRT:

    def __init__(self, minx, maxx, miny, maxy, gs, obs_list, iterations=1000) -> None:
       
        self.minx = minx
        self.maxx = maxx
        self.miny = miny
        self.maxy = maxy
        self.gs = gs
        self.obs_list = obs_list
        self.iterations = iterations
        self.sample_hist = []
        self.goal = None
        self.start = None
        self.tree = None


    def path(self, goal_node: Node):
      
        path_nodes = []
        current_node = goal_node
        while current_node != -1:
            path_nodes.append((current_node.x, current_node.y))
            current_node = current_node.parent_index
        return path_nodes
    
    def is_collision(self, point):
      for obs in self.obs_list:
          if obs.is_inside(cur_x=point.x, cur_y=point.y):
              return True
      return False
    
    def get_new_node(self, tree, random_point):
        nearest_node = None
        min_dist = float('inf')
        for node in tree:
            d = calculate_distance(x1=node.x, x2=random_point.x, y1=node.y, y2=random_point.y)
            if d < min_dist and not self.is_collision(node):
                nearest_node = node
                min_dist = d

        if nearest_node is not None:
            angle = math.atan2(random_point.y - nearest_node.y, random_point.x - nearest_node.x)
            new_x = nearest_node.x + self.gs * math.cos(angle)
            new_y = nearest_node.y + self.gs * math.sin(angle)
            new_node = Node(new_x, new_y)
            new_node.parent_index = nearest_node
            return new_node

        return None
    def run(self, start, goal):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        rrt_tree = [self.start]
        for _ in range(self.iterations):
          random_point = Node(random.uniform(self.minx, self.maxx), random.uniform(self.miny, self.maxy))
          new_node = self.get_new_node(rrt_tree, random_point)
  
          if new_node:
              rrt_tree.append(new_node)
          
          if calculate_distance(x1=new_node.x, y1=new_node.y, x2=self.goal.x, y2=self.goal.y) < self.gs:
              self.goal.parent_index = new_node
              path = self.path(self.goal)
              return path, rrt_tree
          
      
          