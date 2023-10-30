# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 22:23:54 2023

@author: mario
"""

from HW2_Question3 import Dijkstras
from HW2_Q1 import calculate_distance
from HW1_Node import Node, compute_index
from HW2_Question2 import Obstacle, is_valid

class AStartAlgorithm(Dijkstras):
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
        cur_node = Node(x=start[0], y=start[1], cost=0 + calculate_distance(start[0], start[1], goal[0], goal[1]), parent_index=-1)
        cur_node.h_cost = calculate_distance(start[0], start[1], goal[0], goal[1])
        cur_node.g_cost = 0
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
                while cur_node_index != -1:
                    print(self.visited[cur_node_index].g_cost, self.visited[cur_node_index].cost, self.visited[cur_node_index].h_cost, cur_node_index)
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
                new_cost = cur_node.cost + calculate_distance(cur_node.x, cur_node.y, each_neighbour[0], each_neighbour[1]) + calculate_distance(cur_node.x, cur_node.y, goal[0], goal[1])
                
                if self.unvisited.get(idx) != None:
                    if self.unvisited.get(idx).cost > new_cost:
                        self.unvisited[idx].cost = new_cost
                        self.unvisited[idx].parent_index = cur_node_index
                        self.unvisited[idx].g_cost = cur_node.cost + calculate_distance(cur_node.x, cur_node.y, each_neighbour[0], each_neighbour[1])
                        self.unvisited[idx].h_cost = calculate_distance(cur_node.x, cur_node.y, goal[0], goal[1])
                    continue

                self.unvisited[idx] = Node(x=each_neighbour[0],
                                        y=each_neighbour[1],
                                        cost=new_cost,
                                        parent_index=cur_node_index,
                                        h_cost=calculate_distance(cur_node.x, cur_node.y, goal[0], goal[1]),
                                        g_cost=cur_node.cost + calculate_distance(cur_node.x, cur_node.y, each_neighbour[0], each_neighbour[1])
                                        )