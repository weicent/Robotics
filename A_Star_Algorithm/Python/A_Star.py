'''A*寻踪算法'''

import math, random
import numpy as np
import matplotlib.pyplot as plt

# obstacle
top_conner = [50, 50]
bottom_conner = [0, 0]
S = [1, 1]  # start
E = [49, 2]  # end




def generate_boundary(topc, botc):
    # boundary
    ay = list(range(botc[1], topc[1]))  # a边的y坐标
    ax = [botc[0]] * len(ay)  # a边的x坐标
    bx = list(range(botc[0] + 1, topc[0]))
    by = [botc[1]] * len(bx)
    cy = ay + [topc[1]]
    cx = [topc[0]] * len(cy)
    dx = [botc[0]] + bx
    dy = [topc[1]] * len(dx)
    for i in range(300):
        randobx = random.randint(botc[0] + 1, topc[0] - 1)
        randoby = random.randint(botc[1] + 1, topc[1] - 1)
        dx.append(randobx)
        dy.append(randoby)
    boundx = ax + bx + cx + dx
    boundy = ay + by + cy + dy
    bound = np.vstack((boundx, boundy)).T
    bound_list = bound.tolist()
    bound_list.append(S)
    bound_list=[coordinate for coordinate in bound_list if coordinate != S]
    bound_list = [coordinate for coordinate in bound_list if coordinate != E]
    bound = np.array(bound_list)
    return bound


# node
class Node:

    def __init__(self, location, precede=None, G=0, H=0):
        self.precede = precede
        self.G = G
        self.H = H
        self.F = G + H
        self.location = location

    def setg(self, value):
        self.G = value

    def seth(self, value):
        self.H = value

    def setf(self):
        self.F = self.G + self.H

    def setprecede(self, precede):
        self.precede = precede

    def setlocation(self, location):
        self.location = location


# closed list

# 判断是否能走

# 计算曼哈顿距离
def hcost(coor, goal):
    h = abs(coor[0] - goal[0]) + abs(coor[1] - goal[1])
    return h


# 计算G-cost
def gcost(current, next_node):
    dx = abs(current.location[0] - next_node.location[0])
    dy = abs(current.location[1] - next_node.location[1])
    move_cost = math.hypot(dx, dy)
    gcost = current.G + move_cost
    return gcost


# 找出当前节点所有可移动的邻居节点
def find_neibor(node, obstacle):
    neibor = []
    for u in range(node.location[0] - 1, node.location[0] + 2):
        for v in range(node.location[1] - 1, node.location[1] + 2):
            coor = [u, v]
            ob = obstacle.tolist()
            if coor not in ob:
                neibor.append([u, v])
    neibor.remove(node.location)  # bug 报错，neibor里面没有node.location
    return neibor


def find_path(node, start):
    path = np.array([node.location])
    while True:
        path = np.vstack((node.location, path))
        node = node.precede

        if node.location == start.location:
            break
    path = np.vstack((start.location, path[0:-2, :]))
    return path


class Break(Exception):
    pass


if __name__ == '__main__':
    obstacle = generate_boundary(top_conner, bottom_conner)
    OP = []
    CL = []
    closed_point = []
    start = Node(S, H=hcost(S, E))
    end = Node(E)
    dist = [start.H]
    OP.append(start)
    open_point = [start.location]
    try:
        while True:
            for i in range(len(OP)):
                i = 0
                node = OP[0]
                temp = find_neibor(node, obstacle)
                for coor in temp:
                    if coor in closed_point:
                        continue  # 在closed point中的直接跳过
                    elif coor in open_point:  # 在open list中的更新g值
                        upd = Node(coor)
                        g = gcost(node, upd)
                        ind = open_point.index(coor)
                        if g < OP[ind].G:
                            OP[ind].setg(g)
                            OP[ind].setf()

                    else:
                        new = Node(coor)
                        h = hcost(coor, E)
                        dist.append(h)
                        g = gcost(node, new)
                        new.setg(g)
                        new.seth(h)
                        new.setf()
                        new.setprecede(node)

                        OP.append(new)
                        open_point.append(new.location)
                    if dist[-1] == 0:
                        CL.append(new)
                        closed_point.append(node.location)
                        checked = np.array(closed_point)
                        end.setprecede(new)
                        g = gcost(new, end)
                        end.setg(g)
                        end.setf()
                        CL.append(end)
                        raise Break()
                OP.remove(node)
                CL.append(node)
                closed_point.append(node.location)
                checked = np.array(closed_point)
                plt.cla()
                fig=plt.gcf()
                fig.set_size_inches(12,8,forward=True)
                plt.axis('square')
                plt.plot(obstacle[:, 0], obstacle[:, 1], 'xk')
                plt.plot(checked[:, 0], checked[:, 1], 'oy')
                plt.plot(S[0], S[1], '*r')
                plt.plot(E[0], E[1], '*r')
                plt.pause(0.0001)
                OP.sort(key=lambda x: x.F)
                open_point = [point.location for point in OP]
    except Break as e:
        path = find_path(end, start)
        # print(path)
        plt.plot(checked[:, 0], checked[:, 1], 'oy')
        plt.plot(path[:, 0], path[:, 1], '-b')
        plt.show()
