'''A* algorithm
searching path from start and end simultanously'''

import numpy as np
import matplotlib.pyplot as plt
import math


class Node:
    def __init__(self, G=0, H=0, coordinate=[0, 0], parent=None):
        self.G = G
        self.H = H
        self.F = G + H
        self.parent = parent
        self.coordinate = coordinate

    def setg(self, value):
        self.G = value

    def seth(self, value):
        self.H = value

    def setf(self):
        self.F = self.G + self.H

    def set_parent(self, new_parent):
        self.parent = new_parent


class NoPath(Exception):
    '''No path to the goal'''
    pass


class Break(Exception):
    '''Path is find, jump out of loop'''
    pass


def hcost(node_coordinate, goal):
    dx = abs(node_coordinate[0] - goal[0])
    dy = abs(node_coordinate[1] - goal[1])
    hcost = dx + dy
    return hcost


def gcost(fixed_node, update_node_coordinate):
    dx = abs(fixed_node.coordinate[0] - update_node_coordinate[0])
    dy = abs(fixed_node.coordinate[1] - update_node_coordinate[1])
    gc = math.hypot(dx, dy)  # g value of update_node for move from fixed_node to update_node
    gcost = fixed_node.G + gc  # g value for move from start point to update_node
    return gcost


def boundary_and_obstacles(start, goal, topv, botv, obstacle_number):
    '''start for start coordinate
    goal for goal coordinate
    topv for top vertex coordinate of boundary
    botv for bottom vertex coordinate of boundary'''
    ay = list(range(botv[1], topv[1]))
    ax = [botv[0]] * len(ay)
    cy = ay
    cx = [topv[0]] * len(cy)
    bx = list(range(botv[0] + 1, topv[0]))
    by = [botv[1]] * len(bx)
    dx = [botv[0]] + bx + [topv[0]]
    dy = [topv[1]] * len(dx)
    # above can be merged into a rectangle boundary
    ob_x = np.random.randint(botv[0] + 1, topv[0], obstacle_number).tolist()  # generate random obstacles
    ob_y = np.random.randint(botv[1] + 1, topv[1], obstacle_number).tolist()
    x = ax + bx + cx + dx + ob_x  # x coordinate for obstacle and boundary
    y = ay + by + cy + dy + ob_y  # y coordinate for ob and boundary
    bound = np.vstack((x, y)).T
    bound_list = bound.tolist()
    bound_list = [coor for coor in bound_list if coor != start]  # delete start point if include
    bound_list = [coor for coor in bound_list if coor != goal]  # delete goal if include
    bound_list = np.array(bound_list)
    return bound_list


def find_neighbor(node, ob, closed):
    ob_list = ob.tolist()
    neighbor = []
    for x in range(node.coordinate[0] - 1, node.coordinate[0] + 2):
        for y in range(node.coordinate[1] - 1, node.coordinate[1] + 2):
            if [x, y] not in ob_list:
                # find all possible neighbor nodes
                neighbor.append([x, y])
    # remove node violate the motion rule
    # 1. remove node.coordinate itself
    neighbor.remove(node.coordinate)
    # 2. remove neighbor nodes who cross through two diagonal positioned obstacles
    # there is no enough space for robot to go through two diagonal positioned obstacles
    top_nei = [node.coordinate[0], node.coordinate[1] + 1]  # top bottom left right neighbors of node
    bottom_nei = [node.coordinate[0], node.coordinate[1] - 1]
    left_nei = [node.coordinate[0] - 1, node.coordinate[1]]
    right_nei = [node.coordinate[0] + 1, node.coordinate[1]]

    lt_nei = [node.coordinate[0] - 1, node.coordinate[1] + 1]  # left top vertex neighbor
    rt_nei = [node.coordinate[0] + 1, node.coordinate[1] + 1]  # right top vertex neighbor
    lb_nei = [node.coordinate[0] - 1, node.coordinate[1] - 1]  # left bottom vertex neighbor
    rb_nei = [node.coordinate[0] + 1, node.coordinate[1] - 1]  # right bottom vertex neighbor

    if top_nei and left_nei in ob_list and lt_nei in neighbor:
        neighbor.remove(lt_nei)
    if top_nei and right_nei in ob_list and rt_nei in neighbor:
        neighbor.remove(rt_nei)
    if bottom_nei and left_nei in ob_list and lb_nei in neighbor:
        neighbor.remove(lb_nei)
    if bottom_nei and right_nei in ob_list and rb_nei in neighbor:
        neighbor.remove(rb_nei)
    neighbor = [x for x in neighbor if x not in closed]
    return neighbor


def find_node_index(coordinate, node_list):
    # find node index in the node list via its coordinate
    for node in node_list:
        if node.coordinate == coordinate:
            target_node = node
            break
    ind = node_list.index(target_node)
    return ind


def find_path(open_list, closed_list, goal, obstacle):
    dist = []
    flag = len(open_list)
    for i in range(flag):
        node = open_list[0]
        open_coor = [node.coordinate for node in open_list]
        closed_coor = [node.coordinate for node in closed_list]
        temp = find_neighbor(node, obstacle, closed_coor)
        for element in temp:
            if element in closed_list:
                continue
            elif element in open_coor:  # if element in open_coor [i.e. the coordinate list for node in open_list]
                ind = open_coor.index(element)  # find the index for node list by coordinate
                new_g = gcost(node, element)  # calculate new G value
                if new_g < open_list[ind].G:  # if this G value smaller, update node information in open_list
                    open_list[ind].setg(new_g)
                    open_list[ind].setf()
                    open_list[ind].set_parent = node
            else:  # new coordinate, create corresponding node
                ele_node = Node(coordinate=element, parent=node, G=gcost(node, element), H=hcost(element, goal))
                open_list.append(ele_node)
                dist.append(ele_node.H)
        open_list.remove(node)
        closed_list.append(node)
        open_list.sort(key=lambda x: x.H)
    return open_list, closed_list, dist


def draw_plot(close_origin, close_goal, start, end, bound):
    plt.cla()
    plt.gcf().set_size_inches(12, 8, forward=True)
    plt.axis('equal')
    plt.plot(close_origin[:, 0], close_origin[:, 1], 'oy')
    plt.plot(close_goal[:, 0], close_goal[:, 1], 'og')
    plt.plot(bound[:, 0], bound[:, 1], 'sk')
    plt.plot([start[0], end[0]], [start[1], end[1]], '*r')
    plt.pause(0.0001)


def convert_to_coordinate_in_array(node_list):
    coor_list = [node.coordinate for node in node_list]
    coor_array = np.array(coor_list)
    return coor_list, coor_array


def get_path(org_list, goal_list, coor):
    # get origin path
    path_org = []
    path_goal = []
    ind = find_node_index(coor, org_list)
    node = org_list[ind]
    while node != org_list[0]:
        path_org.append(node.coordinate)
        node = node.parent
    path_org.append(org_list[0].coordinate)
    ind = find_node_index(coor, goal_list)
    node = goal_list[ind]
    while node != goal_list[0]:
        path_goal.append(node.coordinate)
        node = node.parent
    path_goal.append(goal_list[0].coordinate)
    path_org.reverse()
    path = path_org + path_goal
    path = np.array(path)
    return path


if __name__ == '__main__':
    try:
        try:
            topv = [60, 60]
            botv = [0, 0]

            start = [np.random.randint(botv[0] + 1, topv[0]),
                     np.random.randint(botv[1] + 1, topv[1])]  # generate start point randomly
            end = [np.random.randint(botv[0] + 1, topv[0]),
                   np.random.randint(botv[1] + 1, topv[1])]  # generate goal randomly
            obstacle_number = 1500

            bound = boundary_and_obstacles(start, end, topv, botv, obstacle_number)

            origin = Node(coordinate=start, H=hcost(start, end))
            goal = Node(coordinate=end, H=hcost(end, start))
            # open list and closed list for search from origin to goal
            origin_open = [origin]
            origin_close = []
            # open list and closed list for search from goal to origin
            goal_open = [goal]
            goal_close = []
            target_origin = start  # target origin is used for searching from end to start
            target_goal = end  # target goal is used for searching from start to end
            while True:
                # searching from start to end
                origin_open, origin_close, origin_dist = \
                    find_path(open_list=origin_open, closed_list=origin_close, goal=target_goal, obstacle=bound)
                if origin_open == []:
                    raise NoPath()

                target_origin = min(origin_open,
                                    key=lambda x: x.H).coordinate  # update target for searching from end to start

                # searching from end to start
                goal_open, goal_close, goal_dist = \
                    find_path(open_list=goal_open, closed_list=goal_close, goal=target_origin, obstacle=bound)
                if goal_open == []:
                    raise NoPath()

                target_goal = min(goal_open,
                                  key=lambda x: x.H).coordinate  # update target for searching from start to end

                # convert node list into coordinate list and array
                org_cor_list, org_cor_array = convert_to_coordinate_in_array(origin_close)
                goa_cor_list, goa_cor_array = convert_to_coordinate_in_array(goal_close)

                og_intersect = [coor for coor in org_cor_list if
                                coor in goa_cor_list]  # check if the searching meet each other

                if og_intersect != []:  # a path is find plot path
                    raise Break()

                draw_plot(org_cor_array, goa_cor_array, start, end, bound)

        except Break as e:
            path = get_path(origin_close, goal_close, og_intersect[0])
            draw_plot(org_cor_array, goa_cor_array, start, end, bound)
            plt.plot(path[:, 0], path[:, 1], '-b')
            plt.title('Robot Arrived', size=20, loc='center')
    except NoPath as e2:
        plt.title('There is no path to goal! Robot&Goal are split by border!', size=20, loc='center')

plt.show()