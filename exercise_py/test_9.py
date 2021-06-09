import math
from os import close
import numpy as np
import matplotlib.pyplot as plt
import time

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
    
    x_max = len(gridmap[0])-1
    y_max = len(gridmap)-1
    
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
    
    print('center')
    print(x, y)
    #print('minimap')
    #[print(row) for row in [gridmap[y-1][x-1:x+2], gridmap[y][x-1:x+2],gridmap[y+1][x-1:x+2],]]
    print('options')
    [print(option) for option in options]
    
    return options

#option(map_example(), (5, 1))


def calc_path(goal_node, closet_set):
    rx, ry = [goal_node.x], [goal_node.y]
    parent = goal_node.parent
    while parent != (-1, -1):
        node = closed_set[parent]
        rx.append(node.x)
        ry.append(node.y)
        parent = node.parent
    return rx, ry

class Node:
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

open_set = dict()
closed_set = dict()


start_node = Node(0, 0, 0, (-1, -1))
goal_node = Node(9, 9, float('inf'), None)


open_set = {}
closed_set = {}

start_id = (start_node.x, start_node.y)
open_set[start_id] = start_node


while True:
    
    
    temp_id = min(open_set, key=lambda o: open_set[o].cost)
    print('temp_id = ', temp_id)
    temp = open_set[temp_id]
    # time.sleep(0.5)
    # plt.plot(temp.x, temp.y, "xc")
    # plt.gcf().canvas.mpl_connect(
    #     'key_release_event',
    #     lambda event: [exit(0) if event.key == 'escape' else None])
    # if len(closed_set.keys()) % 10 == 0:
    #     plt.pause(0.001)
    
    
    
    if temp.x == goal_node.x & temp.y == goal_node.y:
        print('終わり')
        goal_node.parent = temp.parent
        goal_node.cost = temp.cost
        break
    
    
    del open_set[temp_id]
    closed_set[temp_id] = temp
    
    
    for dx, dy, dcost in option(map_example(), temp_id):
        node = Node(temp.x + dx, temp.y + dy, temp.cost + dcost, temp_id)
        node_id = (node.x, node.y)
        
        if node_id in closed_set:
            """決定済みのとき"""
            continue
        
        if node_id not in open_set:
            """未探索のとき"""
            open_set[node_id] = node
        else:
            """未決定のとき"""
            if open_set[node_id].cost >= node.cost:
                open_set[node_id] = node


print('OK!')



rx, ry = calc_path(goal_node, closed_set)

print(rx)
print(ry)
#ry = np.array(ry)
#ry = 10 - ry

map_ = np.array(map_example())
obs = np.where(map_ == 1)
#obsy = 9 - obs[1]

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(rx, ry, marker = '.', color = 'r')
ax.scatter(obs[1], obs[0], color = 'k', marker = '+')

ax.grid(True)
ax.axis('equal')
ax.set_xlim(0, 9)
ax.set_ylim(0, 9)
plt.show()