# -*- coding: utf-8 -*-
"""
Created on Wed Nov 29 21:18:33 2023

@author: mario
"""

import math
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.parent_index = parent_index
        self.cost = cost

def compute_index(min_x, max_x, min_y, max_y, grid_spacing, cx, cy) -> int:

    index = ((cx - min_x)/grid_spacing) + \
            (((cy - min_y)/grid_spacing) * \
            (((max_x + grid_spacing)-min_x)/grid_spacing))

    return int(index)

def calculate_distance(x1, y1, x2, y2):
 
    return round(math.sqrt((x2 - x1)**2 + (y2 - y1)**2), 2)

class Obstacle:
    def __init__(self, x, y, radius=0) -> None:
        self.x = x
        self.y = y
        self.radius = radius

    def is_inside(self, cur_x, cur_y, r_radius=0) -> bool:
     
        dis = calculate_distance(cur_x, cur_y, self.x, self.y)
        if dis > (self.radius + r_radius):
            return False
        return True


def is_valid(obs, x_min, y_min, x_max, y_max, cur_x, cur_y, r_radius=0) -> bool:
  
    for each_obs in obs:
        if each_obs.is_inside(cur_x, cur_y, r_radius):
            return False
        
    if x_min > cur_x:
        return False
    if x_max < cur_x:
        return False
    if y_min > cur_y:
        return False
    if y_max < cur_y:
        return False
    
    return True

class Dijkstras:
   

    def __init__(self, min_x, min_y, max_x, max_y, gs) -> None:
        self.start = None
        self.goal = None
        self.obs_list = None
        self.visited = {}
        self.unvisited = {}
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.gs = gs

    def run(self, start: tuple, goal: tuple, obs_list: list = [], r_radius=0, inflate=False) -> list:
       
        self.visited = {}
        self.unvisited = {}
        self.obs_list = obs_list
        self.goal = goal
        self.start = start
        self.r_radius = r_radius
        # if want to inflate the obstacle or size of the 
        if inflate:
            for i in range(len(self.obs_list)):
                self.obs_list[i].radius = self.obs_list[i].radius * 1.5
            self.r_radius = r_radius * 1.5

        
        goal_index = compute_index(
            self.min_x, self.max_x, self.min_y, self.max_y, self.gs, goal[0], goal[1])
        start_index = compute_index(
            self.min_x, self.max_x, self.min_y, self.max_y, self.gs, start[0], start[1])
        cur_node = Node(x=0, y=0, cost=0, parent_index=-1)
        self.unvisited[0] = cur_node
        while (cur_node.x, cur_node.y) != goal:
            cur_node_index = min(
                self.unvisited, key=lambda x: self.unvisited[x].cost)
            cur_node = self.unvisited[cur_node_index]
            self.visited[cur_node_index] = cur_node

            del self.unvisited[cur_node_index]
            # if current node is the goal location the break our of the loop
            if (cur_node.x, cur_node.y) == goal:
                route = []
                print(cur_node.cost)
                while cur_node_index != -1:
                    route.append([self.visited[cur_node_index].x, self.visited[cur_node_index].y])
                    cur_node_index = self.visited[cur_node_index].parent_index

                return route[::-1]

            all_neighbours = self.get_neighbour_moves(
                cur_x=cur_node.x, cur_y=cur_node.y)
            all_neighbours = list(filter(lambda x: is_valid(obs=self.obs_list,
                                                            x_min=self.min_x,
                                                            x_max=self.max_x,
                                                            y_min=self.min_y,
                                                            y_max=self.max_y,
                                                            cur_x=x[0],
                                                            cur_y=x[1],
                                                            r_radius=r_radius
                                                            ), all_neighbours))
            for each_neighbour in all_neighbours:
                idx = compute_index(
                    self.min_x, self.max_x, self.min_y, self.max_y, self.gs, each_neighbour[0], each_neighbour[1])
                if self.visited.get(idx) != None:
                    continue
                new_cost = cur_node.cost + calculate_distance(cur_node.x, cur_node.y, each_neighbour[0], each_neighbour[1])
                if self.unvisited.get(idx) != None:
                    if self.unvisited.get(idx).cost > new_cost:
                        self.unvisited[idx].cost = new_cost
                        self.unvisited[idx].parent_index = cur_node_index
                    continue

                self.unvisited[idx] = Node(x=each_neighbour[0],
                                        y=each_neighbour[1],
                                        cost=new_cost,
                                        parent_index=cur_node_index
                                        )
                

    def get_neighbour_moves(self, cur_x, cur_y):
        import numpy as np
       
        neighbours = []
        for each_x in np.arange(-self.gs, self.gs + self.gs, self.gs):
            for each_y in np.arange(-self.gs, self.gs + self.gs, self.gs):
                if (cur_x == (each_x + cur_x) and cur_y == (each_y + cur_y)):
                    continue
                neighbours.append([(each_x + cur_x), (each_y + cur_y)])
        return neighbours

    def plot_route(self, route):
  
        import matplotlib.pyplot as plt
        import numpy as np
        fig, ax = plt.subplots()

        x_values = np.arange(self.min_x, self.max_x + self.gs, self.gs)
        y_values = np.arange(self.min_y, self.max_y + self.gs, self.gs)

        for y in y_values:
            for x in x_values:
                index = compute_index(self.min_x, self.max_x, self.min_y, self.max_y, self.gs, x, y)
                plt.text(x, y, str(index), color='red', fontsize=8, ha='center', va='center')

        for obs in self.obs_list:
            obs_plot = plt.Circle((obs.x, obs.y), obs.radius, color='black')
            ax.add_patch(obs_plot)
        obs_plot = plt.Circle((self.goal[0], self.goal[1]), self.r_radius, color='green')
        ax.add_patch(obs_plot)
        plt.plot([x[0] for x in route], [x[1] for x in route], c='red')
        plt.xlim(self.min_x - self.gs, self.max_x + self.gs)
        plt.ylim(self.min_y - self.gs, self.max_y + self.gs)
        plt.show()

if __name__ == "__main__":
    obs_pos = [(1, 1), (4, 4), (3, 4), (5, 0), (5, 1),
               (0, 7), (1, 7), (2, 7), (3, 7)]
    obs_radius = 0.25
    obs_list = [Obstacle(each_ob[0], each_ob[1], obs_radius)
                for each_ob in obs_pos]
    djs = Dijkstras(0, 0, 10, 10, 0.5)
    route = djs.run(start=(0, 0), goal=(8, 9), r_radius=0, obs_list=obs_list)
    djs.plot_route(route=route)