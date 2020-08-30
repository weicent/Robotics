'''A*寻踪算法'''
'''后续可以加入起点和终点一起搜索的优化，当前代码只有从起点搜索'''
import math, random
import numpy as np
import matplotlib.pyplot as plt


def generate_boundary(S, E, topc, botc, obstacle_number):
    # 生成边界以及边界内的随机障碍物数目
    ay = list(range(botc[1], topc[1]))  # a边的y坐标
    ax = [botc[0]] * len(ay)  # a边的x坐标
    bx = list(range(botc[0] + 1, topc[0]))
    by = [botc[1]] * len(bx)
    cy = ay + [topc[1]]
    cx = [topc[0]] * len(cy)
    dx = [botc[0]] + bx
    dy = [topc[1]] * len(dx)
    # 以上列表可以合成四边形边界
    for i in range(obstacle_number):  # 循环产生在边界内的随机数障碍物
        randobx = random.randint(botc[0] + 1, topc[0] - 1)
        randoby = random.randint(botc[1] + 1, topc[1] - 1)
        dx.append(randobx)
        dy.append(randoby)

    boundx = ax + bx + cx + dx
    boundy = ay + by + cy + dy
    bound = np.vstack((boundx, boundy)).T  # 合成边界与障碍物
    bound_list = bound.tolist()  # 转换列表去除生成的起点和终点
    bound_list = [coordinate for coordinate in bound_list if coordinate != S]
    bound_list = [coordinate for coordinate in bound_list if coordinate != E]
    bound = np.array(bound_list)  # 转换回数组方便画图
    return bound


# node
class Node:
    '''节点类，包含每个节点位置信息，移动代价G-cost，曼哈顿距离H，和值F以及父节点'''

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


def find_neibor(node, obstacle):
    ob = obstacle.tolist()  # 转换列表便于进行坐标检查，【数组检查单独数值，而不会核对坐标】
    neibor = []
    # 当前节点node上下左右节点的坐标
    ob1 = [node.location[0], node.location[1] + 1]
    ob2 = [node.location[0] - 1, node.location[1]]
    ob3 = [node.location[0], node.location[1] - 1]
    ob4 = [node.location[0] + 1, node.location[1]]
    # 生成相邻节点
    for u in range(node.location[0] - 1, node.location[0] + 2):
        for v in range(node.location[1] - 1, node.location[1] + 2):
            coor = [u, v]
            if coor not in ob:  # 有障碍物的地方不能生成邻节点
                neibor.append([u, v])
    # 相邻的四个顶角节点
    c1 = [node.location[0] - 1, node.location[1] + 1]
    c2 = [node.location[0] + 1, node.location[1] + 1]
    c3 = [node.location[0] - 1, node.location[1] - 1]
    c4 = [node.location[0] + 1, node.location[1] - 1]
    # 两个障碍物之间的缝隙距离太短，不能穿行，因此移除该邻节点坐标
    if ob1 and ob2 in ob and c1 in neibor:
        # 该逻辑为，如果上（ob1）和左（ob2）有障碍物，且生成的相邻节点列表neibor中有了对应顶角节点坐标
        # 那么移除这个顶角节点坐标，因为不能从两个斜着摆放的障碍物之间穿行
        neibor.remove(c1)
    if ob1 and ob4 in ob and c2 in neibor:
        neibor.remove(c2)
    if ob2 and ob3 in ob and c3 in neibor:
        neibor.remove(c3)
    if ob3 and ob4 in ob and c4 in neibor:
        neibor.remove(c4)
    neibor.remove(node.location)  # 去除当前节点本身
    return neibor


def find_path(node, start):
    path = np.array([node.location])  # 指定path的类型与数组大小
    while True:
        path = np.vstack((node.location, path))  # 堆叠
        node = node.precede  # node指针指向它的父节点
        if node.location == start.location:  # 找到父节点，跳出循环体
            break
    path = np.vstack((start.location, path[0:-2, :]))  # path在传入的时候end重复堆叠，去掉多余的
    return path


class Break(Exception):
    '''找到路径，跳出循环体'''
    pass


class NoPath(Exception):
    '''没有路径，跳出循环体'''
    pass


if __name__ == '__main__':
    '''主程序开始'''
    top_conner = [60, 60]  # 边界上顶点-位置在右上
    bottom_conner = [-10, -10]  # 边界下顶点-位置在左下
    S = [random.randint(bottom_conner[0]+1,top_conner[0]-1),\
         random.randint(bottom_conner[1]+1,top_conner[1]-1)]#[1, 49]  # 起始点
    E = [random.randint(bottom_conner[0]+1,top_conner[0]-1),\
         random.randint(bottom_conner[1]+1,top_conner[1]-1)]#[49, 1]  # 目标点
    obstacle_number = 1000  # 生成的障碍物坐标数目，越大寻踪难度越大，耗时越久
    obstacle = generate_boundary(S, E, top_conner, bottom_conner, obstacle_number)  # 生成障碍与边界
    OP = []  # 节点的open list
    CL = []  # 节点的closed list
    closed_point = []  # 节点坐标的 closed list
    start = Node(S, H=hcost(S, E))  # 将开始坐标生成node实例
    end = Node(E)  # 将目标点生成node实例
    dist = [start.H]  # 距离终点的曼哈顿距离列表
    OP.append(start)
    open_point = [start.location]  # 节点坐标的open list
    try:
        try:
            while True:
                for i in range(len(OP)):  # 循环OP列表中的待检查节点
                    # i = 0  # 不断将i归零，在后续代码中会对OP进行排序，使OP[0]总有最小的H值，以提高寻踪效率
                    node = OP[0]
                    temp = find_neibor(node, obstacle)  # 生成相邻节点
                    if temp == []:
                        # temp空集，由于find_neibor函数允许返回closed point，即可以原路返回，
                        # 所以一旦temp空集便表示传入node为空，代表已遍历所有可行路径，寻踪失败，
                        # 随机生成的障碍物全是死路，没有通往目标的路径
                        raise NoPath()
                    for coor in temp:  # 对邻节点遍历检查
                        if coor in closed_point:
                            continue  # 在closed point中的已检查节点直接跳过
                        elif coor in open_point:  # 在open list中的更新g值
                            upd = Node(coor)
                            g = gcost(node, upd)
                            ind = open_point.index(coor)
                            if g < OP[ind].G:
                                OP[ind].setg(g)
                                OP[ind].setf()
                        else:  # 全新节点计算G值和曼哈顿距离H，添加父节点等，加入OP列表
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
                        if dist[-1] == 0:  # 如果当前节点的h值等于0，则到达终点，结束循环
                            CL.append(new)
                            closed_point.append(node.location)
                            checked = np.array(closed_point)
                            end.setprecede(new)
                            g = gcost(new, end)
                            end.setg(g)
                            end.setf()
                            CL.append(end)
                            raise Break()
                    OP.remove(node)  # 当前节点已完成检查，将之从OP列表中剔除
                    CL.append(node)  # 节点加入CL列表
                    closed_point.append(node.location)
                    checked = np.array(closed_point)  # 这个用来画图，否则每次循环画图都要提取一遍CL中所有节点的location属性
                    # 画图
                    plt.cla()
                    fig = plt.gcf()
                    fig.set_size_inches(12, 8, forward=True)
                    plt.axis('square')
                    plt.plot(obstacle[:, 0], obstacle[:, 1], 'sk')
                    plt.plot(checked[:, 0], checked[:, 1], 'oy')
                    plt.plot(S[0], S[1], '*g')
                    plt.plot(E[0], E[1], '*r')
                    plt.pause(0.0001)
                    OP.sort(key=lambda x: x.H)
                    open_point = [point.location for point in OP]
                    # OP空集还没有找到路径，没有路径
                    if OP == []:
                        raise NoPath()
        except Break as e:
            # 寻踪结束，画出路径
            path = find_path(end, start)
            plt.plot(path[:, 0], path[:, 1], '-b')
            plt.title('Robot arrived!', size=20, loc='center')
            plt.plot(S[0], S[1], '*r')
            plt.show()
    except NoPath as e2:
        # 寻踪失败，打印路径
        plt.title('There is no path to the goal!', size=20, loc='center')
        plt.show()
