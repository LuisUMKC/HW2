# -*- coding: utf-8 -*-
"""
Created on Mon Sep 11 14:39:13 2023

@author: mario
"""

from HW2_Question2 import Obstacle, is_not_valid
from HW2_Question1 import distance
from HW1_Node import Node, compute_index

class Dijkstras:
    def __init__(self, min_x, min_y, max_x, max_y, gs) -> None:
        self.start = None
        self.end = None 
        self.path_blocked = None
        self.visited = {} 
        self.unvistited = {} 
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x 
        self.max_y = max_y 
        self.gs = gs
    
    def run(self, start:list, end:list, path_blocked:list = [], robot_radius=0) -> list:
        self.visited = {} 
        self.unvistited = {}
        self.path_blocked = path_blocked
        self.end = end 
        self.start = start
        self.robot_radius = robot_radius
    
                
                
                
        goal_index = compute_index(self.min_x, self.max_x, self.min_y, self.max_y, self.gs, end[0], end[1])
        start_index = compute_index(self.min_x, self.max_x, self.min_y, self.max_y, self.gs, start[0], start[1])
        curr_node = Node(x=0, y=0, parent_cost=0, index=1)
        self.unvistited[0] = curr_node
        while (curr_node.x, curr_node.y) != end:
            curr_node_index = min(self.unvistited, key=lambda x: self.unvistited[x].parent_cost)
            curr_node = self.unvistited[curr_node_index]
            self.visited[curr_node_index] = curr_node   
            
            del self.unvistited[curr_node_index] 
            if (curr_node.x, curr_node.y) == end:
                route = [] 
                while curr_node_index != -1:
                    route.append([self.visited[curr_node_index].x, self.visited[curr_node_index].y])
                    curr_node_index = self.visited[curr_node_index].index
                    return route [::-1] 
                
                all_neighbors = self.get_neighbor_moves(curr_x=curr_node.x, curr_y=curr_node.y)
                all_neighbors = list(filter(lambda x: is_not_valid(obs=self.path_blocked, x_min=self.min_x, y_min=self.min_y, x_max=self.max_x, y_max=self.max_y, curr_x=x[0], curr_y=x[1], robot_radius=robot_radius), all_neighbors))
               
                for each_neighbor in all_neighbors:
                    idx = compute_index(self.min_x, self.max_x, self.min_y, self.max_y, self.gs, each_neighbor[0], each_neighbor[1])
                    if self.visited.get(idx) != None:
                        continue 
                    new_cost = curr_node.cost + distance(curr_node.x, curr_node.y, each_neighbor[0], each_neighbor[1])
                    if self.visited.get(idx) != None:
                        if self.unvistited.get(idx).cost > new_cost:
                            self.unvistited[idx].cost = new_cost
                            self.unvistited[idx].index = curr_node_index
                        continue
                    self.unvistited[idx] = Node(x=each_neighbor[0], y=each_neighbor[1], cost=new_cost)
                    
    def get_neighbor_moves(self, curr_x, curr_y) :
        import numpy as np
        neighbors = []
        for each_x in np.arange(-self.gs, self.gs + self.gs, self.gs):
            for each_y in np.arange(-self.gs, self.gs + self.gs, self.gs):
                if (curr_x == (each_x + curr_x) and curr_y == (each_y + curr_y)):
                    continue 
                neighbors.append([(each_x + curr_x), (each_y + curr_y)])
            return neighbors 
        
    def plot_route(self, route):
        import matplotlib.pyplot as plt
        import numpy as np 
        fig, ax = plt.subplots() 
        
        x_values = np.arange(self.min_x, self.max_x + self.gs, self.gs)
        y_values = np.arange(self.min_y, self.max_y + self.gs, self.gs) 
        
        for y in y_values:
            for x in x_values: 
                index = compute_index(self.min_x, self.max_x, self.min_y, self.max_y, self.gs, x, y)
                plt.text(x, y, str(index), color='red', fontsize=5, ha='center', va='center')
                
        for obs in self.path_blocked:
            obs_plot = plt.Circle((obs.x, obs.y), obs.radius, color='black')
            ax.add_patch(obs_plot)
        obs_plot = plt.Circle((self.end[0], self.end[1]), self.robot_radius + 0.25, color='green')
        ax.add_patch(obs_plot)
        plt.plot([x[0] for x in route], [x[1] for x in route], c='red')
        plt.xlim(self.min_x - self.gs, self.max_x + self.gs)
        plt.ylim(self.min_y - self.gs, self.max_y + self.gs)
        plt.show() 
        
if __name__ == "__main__":
    obs_pos = [(1,1), (4,4), (3,4), (5,0), (5,1), (0,7), (1,7), (2,7), (3,7)]
    obs_radius = 0.25
    path_blocked = [Obstacle(each_ob[0], each_ob[1], obs_radius) for each_ob in obs_pos]
    djs = Dijkstras(0, 0, 10, 10, 0.5)
    route = djs.run(start=(0,0), end=(8, 9), robot_radius=0, path_blocked=path_blocked)
    djs.plot_route(route=route)
        