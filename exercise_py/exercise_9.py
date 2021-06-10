#!/usr/bin/env python

import math
import numpy as np
import matplotlib.pyplot as plt
import cv2



class Node:
    """node"""
    
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent


class Dijkstra:
    """dijkstra"""
    
    def __init__(self, gridmap,):
        """
        Parameters
        ---
        gridmap : ndarray
            map
        """
        self.gridmap = gridmap
    
    
    def option(self, id):
        """移動可能なノードを計算
        
        Parameters
        ---
        id : tuple
            (x, y)
        
        Returns
        ---
        out : list
            [dx, dy, dcost]
        """
        
        x, y = id
        x_max = self.gridmap.shape[1] - 1
        y_max = self.gridmap.shape[0] - 1
        
        options = []
        
        if x + 1 <= x_max:
            if y + 1 <= y_max and not self.gridmap[y + 1, x + 1]:
                options.append([1, 1, math.sqrt(2)])
            if y - 1 >= y_max and not self.gridmap[y - 1, x + 1]:
                options.append([1, -1, math.sqrt(2)])
            if not self.gridmap[y, x + 1]:
                options.append([1, 0, 1])
        if x - 1 >= 0:
            if y + 1 <= y_max and not self.gridmap[y + 1, x - 1]:
                options.append([-1, 1, math.sqrt(2)])
            if y - 1 >= 0 and not self.gridmap[y - 1, x - 1]:
                options.append([-1, -1, math.sqrt(2)])
            if not self.gridmap[y, x-1]:
                options.append([-1, 0, 1])
        else:
            if y + 1 <= y_max and not self.gridmap[y + 1, x]:
                options.append([0, 1, 1])
            if y - 1 >= 0 and not self.gridmap[y - 1, x]:
                options.append([0, -1, 1])
        
        return options
    
    
    def compute_optiomal_path(self, goal_node, closed_set):
        """startからgoalへの最短距離を計算
        
        Prameters
        ---
        goal_node : class
            node class
        closed_set : dict
            ***
        
        Returns
        ---
        out : tuple
            rx, ry : optiomal x,y sequence. cost : goukei of cost.
        """
        
        rx, ry = [goal_node.x], [goal_node.y]
        parent = goal_node.parent
        cost = goal_node.cost
        while parent != (-1, -1):
            node = closed_set[parent]
            rx.append(node.x)
            ry.append(node.y)
            cost += node.cost
            parent = node.parent
        
        return rx, ry, cost
    
    
    def planning(self, start, goal):
        """プランニング本体
        
        Parameters
        ---
        start : tuple
            start position
        goal : tuple
            goal position
        
        Returns
        ---
        out : tuple
            (rx, ry)
        """
        
        start_node = Node(start[0], start[1], 0, (-1, -1))
        goal_node = Node(goal[0], goal[1], float('inf'), None)
        
        open_set = {}
        closed_set = {}
        
        start_id = (start_node.x, start_node.y)
        open_set[start_id] = start_node
        
        while True:
            #print(len(open_set))
            temp_id = min(open_set, key=lambda o: open_set[o].cost)
            #print('temp_id = ', temp_id)
            temp = open_set[temp_id]
            
            if temp.x == goal_node.x & temp.y == goal_node.y:
                print('終わり')
                goal_node.parent = temp.parent
                goal_node.cost = temp.cost
                break
            
            
            del open_set[temp_id]
            closed_set[temp_id] = temp
            
            
            for dx, dy, dcost in self.option(temp_id):
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
        
        rx, ry, cost = self.compute_optiomal_path(goal_node, closed_set)
        
        return rx, ry, cost, closed_set
    
    
    def draw(self, rx, ry):
        """図示"""
        
        obs = np.where(self.gridmap == 1)
        
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(rx, ry, marker = '.', color = 'r', label = 'optiomal path')
        ax.scatter(obs[1], obs[0], color = 'k', marker = '+', label = 'obstacle')
        ax.scatter(rx[0], ry[0], marker = '*', color = 'r', label = 'goal')
        ax.scatter(rx[-1], ry[-1], marker = 'o', color = 'g', label = 'start')
        
        ax.grid(True)
        #ax.axis('equal')
        ax.set_xlim(-1, self.gridmap.shape[1])
        ax.set_ylim(-1, self.gridmap.shape[0])
        
        plt.show()
        
        return




def main():
    map_example = [
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
    
    map_example = np.array(map_example)
    
    simu = Dijkstra(map_example)
    rx, ry, cost, _ = simu.planning((0, 0), (9, 9))
    print('cost = ', cost)
    simu.draw(rx, ry)
    
    return



if __name__ == '__main__':
    main()


