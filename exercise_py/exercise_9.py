#!/usr/bin/env python

import os
import pathlib
import math
import numpy as np
import matplotlib.pyplot as plt
import cv2


class Node:
    """node"""
    
    def __init__(self, x, y, cost, parent):
        """
        Parameters
        ---
        x : int
            node position x
        y : int
            node position y
        cost : float
            cost
        parent : tuple[int, int]
            parent index
        """
        
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
        gridmap : list[list, ..., list]
            map
        """
        self.gridmap = gridmap
        self.x_max = len(self.gridmap[0]) - 1
        self.y_max = len(self.gridmap) - 1
    
    
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

        
        options = []
        
        if x + 1 <= self.x_max:
            if y + 1 <= self.y_max and not self.gridmap[y + 1][x + 1]:
                options.append([1, 1, math.sqrt(2)])
            if y - 1 >= 0 and not self.gridmap[y - 1][x + 1]:
                options.append([1, -1, math.sqrt(2)])
            if not self.gridmap[y][x + 1]:
                options.append([1, 0, 1])
        if x - 1 >= 0:
            if y + 1 <= self.y_max and not self.gridmap[y + 1][x - 1]:
                options.append([-1, 1, math.sqrt(2)])
            if y - 1 >= 0 and not self.gridmap[y - 1][x - 1]:
                options.append([-1, -1, math.sqrt(2)])
            if not self.gridmap[y][x-1]:
                options.append([-1, 0, 1])
        else:
            if y + 1 <= self.y_max and not self.gridmap[y + 1][x]:
                options.append([0, 1, 1])
            if y - 1 >= 0 and not self.gridmap[y - 1][x]:
                options.append([0, -1, 1])
        
        return options
    
    
    def planning(self, start_position, goal_position):
        """プランニング本体
        
        Parameters
        ---
        start : tuple[float, float]
            スタート位置( x0, y0)
        goal : tuple[float, float]
            ゴール位置(xg, yg)
        
        Returns
        ---
        out : tuple[lisy, lisy, float, dict]
            最短経路(rx, ry).rx:xのリスト，ry:yのリスト
        """
        
        self.start_position = start_position
        self.goal_position = goal_position
        
        start_node = Node(start_position[0], start_position[1], 0, None)
        goal_node = Node(goal_position[0], goal_position[1], float('inf'), None)
        
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
        #print('optiomal path')
        rx, ry = [goal_node.x], [goal_node.y]
        parent = goal_node.parent
        cost = goal_node.cost
        
        while parent != self.start_position:
            node = closed_set[parent]
            rx.append(node.x)
            ry.append(node.y)
            cost += node.cost
            parent = node.parent
            #print(node.x, node.y)
        
        return rx, ry, cost
    
    def draw(self, rx, ry, rawmap, dilmap=None, save=False):
        """図示
        
        Parameters
        ---
        rx : list[int]
            最短経路のx列
        ry : list[int]
            最短経路のy列
        rawmap : list[list[bool], ..., list[bool]]
            生マップ
        dilmap : list[list[bool], ..., list[bool]]
            安全半径ありマップ
        """
        
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(
            rx, ry,
            color = 'r', label = 'optiomal path'
        )
        
        gridmap_ = np.array(rawmap)
        obs = np.where(gridmap_ == 1)
        ax.scatter(obs[1], obs[0], color = 'k', marker = '+', label = 'obstacle')
        
        if dilmap is not None:
            dilmap_ = np.array(dilmap)
            obs_ = np.where(dilmap_ == 1)
            ax.scatter(
                obs_[1], obs_[0],
                color = 'b', marker = '+', label = 'obs expansion', alpha = 0.1
            )
        
        ax.scatter(rx[0], ry[0], marker = '*', color = 'r', label = 'goal', s = 50)
        ax.scatter(rx[-1], ry[-1], marker = 'o', color = 'g', label = 'start', s = 50)
        
        ax.grid(True)
        ax.set_aspect('equal')
        ax.set_xlim(-1, self.x_max + 1)
        ax.set_ylim(-1, self.y_max + 1)
        ax.legend()
        plt.show()
        
        if save:
            fig.savefig('exercise_9.png')
        
        return


def make_gridmap(path_map,):
    """make gridmap(2-d list) from image
    
    ＊最短経路問題とは直接関係ない
    
    Parameters
    ---
    path_map : str
        imgのパス
    
    Returns
    ---
    out : tiple[list[list[bool], ..., list[bool]], list[list[bool], ..., list[bool]]]
        生マップ，拡張マップ
    """
    
    map_img = cv2.imread(path_map, 0)
    _, map_img_01 = cv2.threshold(
        map_img, 0, 255, cv2.THRESH_OTSU
    )
    raw_gridmap = np.array(map_img_01) < 255
    
    # dilation
    img = cv2.bitwise_not(map_img_01)
    kernel = np.ones((9,9), np.uint8)
    map_img_dil = cv2.dilate(
        img, kernel, iterations = 1
    )
    dil_gridmap = np.array(map_img_dil) > 0
    
    return raw_gridmap, dil_gridmap


def exercise_9_main():
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
    
    simu = Dijkstra(map_example)
    rx, ry, cost, _ = simu.planning((0, 0), (9, 9))
    print('cost = ', cost)
    simu.draw(rx, ry, map_example)
    
    return


def exercise_9_main_2():
    rawmap, dilmap = make_gridmap('./misc/gibbons_mainfloor_map.png')
    
    simu = Dijkstra(dilmap)
    rx, ry, cost, _ = simu.planning((0, 0), (230, 230))
    print('cost = ', cost)
    simu.draw(rx, ry, rawmap, dilmap, save=False)
    
    return


if __name__ == '__main__':
    #exercise_9_main()
    
    exercise_9_main_2()


