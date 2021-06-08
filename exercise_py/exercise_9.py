#!/usr/bin/env python

import math
import numpy as np
import matplotlib.pyplot as plt
import cv2


def map_example():
    map = [
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 1, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 1, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    ]  # mapの例．0は通過可能な点，1は通貨不可能な点を示す
    return map

def one_step():
    
    z = [
        [1, 0, 1],
        [0, 1, 1],
        [-1, 0, 1],
        [0, -1, 1],
        [-1, -1, math.sqrt(2)],
        [-1, 1, math.sqrt(2)],
        [1, -1, math.sqrt(2)],
        [1, 1, math.sqrt(2)],
    ]
    return z


class Node:
    """node"""
    
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent


class Dijkstra:
    """dijkstra"""
    
    def __init__(self, map, start, goal):
        
        self.map = map
        self.start = start
        self.goal = goal
    
    
    def planning(self,):
        
        start_node = Node(x=self.start[0], y=self.start[1], cost=0, parent=-1)
        goal_node = Node(x=self.goal[0], y=self.goal[1], cost=0, parent=-1)
        
        open_set = dict()
        closed_set = dict()
        
        open_set[(start_node.x, start_node.y)] = start_node
        
        while True:
            
            temp_id = min(open_set, key=lambda o: open_set[o].cost)
            temp = open_set[temp_id]
            
            if temp.x == self.goal.x & temp.y == self.goal.y:
                print('end node')
                goal_node.parent = temp.parent
                goal_node.cost = temp.cost
                break
            
            del open_set[temp_id]
            closed_set[temp_id] = temp
            
            
            for x_, y_, cost_ in one_step():
                node = Node(temp.x + x_, temp.y + y_, temp.cost + cost_, temp_id)
                node_id = 
    





if __name__ == '__main__':
    hoge = Dijkstra(map_example(), (0, 0), (10, 0))
    hoge.planning()
