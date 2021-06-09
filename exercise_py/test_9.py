import math
from os import close
import numpy as np
import matplotlib.pyplot as plt


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

def option(gridmap, id):
    
    x, y = id
    # x_max = map.shape[1]
    # y_max = map.shape[0]
    
    x_max = len(gridmap[0])
    y_max = len(gridmap)
    
    options = []
    
    if x+1 <= x_max and y+1 <= y_max:
        if not gridmap[y+1][x+1]:
            options.append([1, 1, math.sqrt(2)])
    if y+1 <= y_max:
        if not gridmap[y+1][x]:
            options.append([0, 1, 1])
    if x-1 >= 0 and y+1 <= y_max:
        if not gridmap[y+1][x-1]:
            options.append([-1, 1, math.sqrt(2)])
    if x-1 >= 0:
        if not gridmap[y][x-1]:
            options.append([-1, 0, 1])
    if x-1 >= 0 and y-1 >= 0:
        if not gridmap[y-1][x-1]:
            options.append([-1, -1, math.sqrt(2)])
    if y-1 >= 0:
        if not gridmap[y-1][x]:
            options.append([0, -1, 1])
    if x+1 <= x_max and y-1 >= y_max:
        if not gridmap[y-1][x+1]:
            options.append([1, -1, math.sqrt(2)])
    if x+1 <= x_max:
        if not gridmap[y][x+1]:
            options.append([1, 0, 1])
    
    # print('minimap')
    # [print(row) for row in [gridmap[y-1][x-1:x+2], gridmap[y][x-1:x+2],gridmap[y+1][x-1:x+2],]]
    # print('options')
    # [print(option) for option in options]
    
    return options

option(map_example(), (5, 1))


class Node:
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

open_set = dict()
closed_set = dict()


start_node = Node(0, 0, 0, -1)
goal_node = Node(9, 9, float('inf'), None)


open_set = {}
closed_set = {}

start_id = (start_node.x, start_node.y)
open_set[start_id] = start_node


# for i in range(1):
    
#     temp_id = min(open_set, key=lambda o: open_set[o].cost)
#     temp = open_set[temp_id]
    
#     if temp.x == goal_node.x & temp.y == goal_node.y:
#         print('終わり')
#         goal_node.parent = temp.parent
#         goal_node.cost = temp.cost
#         break
    
    
#     del open_set[temp_id]
#     closed_set[temp_id] = temp
    
    
#     for x, y, cost in option(map_example(), temp_id):
        
        
