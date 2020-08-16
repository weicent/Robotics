import numpy as np
import math
import matplotlib.pyplot as plt


class Parameter:
    def __init__(self):
        self.x_init = [0, 0, 0, 0, 0]
        self.u_init = [0, 0]
        self.Vmax = 1
        self.Vmin = -0.5
        self.Wmax = 40 * math.pi / 180  # 角速度
        self.Wmin = -40 * math.pi / 180
        self.vv = 0.2  # m/s**2
        self.ww = 40 * math.pi / 180  # rad/s**2
        self.interval_v = 0.01  # 动态取值分辨率
        self.interval_w = 0.1 * math.pi / 180
        self.dt = 0.1  # 时间步长
        self.predict_T = 3  # 窗口预测时间长度
        self.Heading = 0.15  # 评价函数系数，用作校正权重，分别为角度系数，速度系数，障碍距离系数
        self.Velocity = 1
        self.Obstacle = 1
        self.ob = np.array([[-1, -1],
                            [0, 2],
                            [4.0, 2.0],
                            [3.0, 4.0],
                            [6.0, 5.0],
                            [5.0, 4.0],
                            [5.0, 9.0],
                            [8.0, 9.0],
                            [7.0, 9.0],
                            [8.0, 10.0],
                            [9.0, 8.0],
                            [10.0, 13.0],
                            [12.0, 12.0],
                            [15.0, 15.0],
                            [13.0, 13.0]
                            ])
        self.robot_radius = 1
        self.rObstacle=1
        self.goal = [10, 10]


def dynamic_window(x,traj):

    #机器人能达到的最大速度
    vmin = x[3] - para.vv * para.dt
    vmax = x[3] + para.vv * para.dt
    wmin = x[4] - para.ww * para.dt
    wmax = x[4] + para.ww * para.dt

    #确保能够及时在障碍物前刹车的最大速度
    break_v=math.sqrt(2*dist_to_nearest_obstacle(traj)*para.vv)
    break_w=math.sqrt(2*dist_to_nearest_obstacle(traj)*para.ww)

    dw = [max(para.Vmin, vmin), min(para.Vmax, vmax, break_v), \
          max(para.Wmin, wmin), min(para.Wmax, wmax, break_w)]
    return dw


def motion(x, u):
    x[2] += u[1] * para.dt
    x[0] += u[0] * para.dt * math.cos(x[2])
    x[1] += u[0] * para.dt * math.sin(x[2])
    x[3] = u[0]
    x[4] = u[1]
    return x


def predict_trajactory(x_init, u):
    x = np.array(x_init)  # 注意这里的问题！！！！
    # x=x_init #使用列表类型会导致轨迹错乱无序
    '''np.array存储数据类型，对内部数据直接计算，而list列表并不存储数据，只存储指针指向
    此处如果用x=x_init列表类型，x指向的列表数据被传入motion并不断操作，经过多轮迭代后指
    针指向的数据本身早已被改变，每次x的列表复位都只是复位指针指向的内存地址，而并非数据本身
    因此使用x=x_init类型轨迹会十分混乱无序。
    而另一方面，np.array内部直接存储数据，每次迭代都是对数据本身操作，迭代完成后数组的复位
    都是数值本身的复位，而非指针指向'''
    time = 0
    trajactory = np.array(x)
    while time < para.predict_T:
        time += para.dt
        x = motion(x, u)
        trajactory = np.vstack((trajactory, x))
    return trajactory

def dist_to_nearest_obstacle(predicted_trajactory):
    dx = predicted_trajactory[:, 0] - para.ob[:, 0][:, None]
    dy = predicted_trajactory[:, 1] - para.ob[:, 1][:, None]
    r = np.hypot(dx, dy)

    return np.min(r)

def cost(predicted_trajactory):
    # dx = predicted_trajactory[-1, 0] - para.goal[0]
    # dy = predicted_trajactory[-1, 1] - para.goal[1]
    # angle_diff = abs(math.atan2(dy, dx))

    dx = para.goal[0] - predicted_trajactory[-1, 0]
    dy = para.goal[1] - predicted_trajactory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - predicted_trajactory[-1, 2]
    angle_diff = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    speed_diff = para.Vmax - predicted_trajactory[-1, 3]

    r=dist_to_nearest_obstacle(predicted_trajactory)

    if r <= para.robot_radius:
        obstacle = float('inf')
    else:
        obstacle = 1 /r
    total_cost = para.Heading * angle_diff + para.Velocity * speed_diff + \
                 para.Obstacle * obstacle
    return total_cost


def traj_filter(x, dw):
    min_cost = float('inf')
    best_traj = np.array(x)
    best_u = [0, 0]
    for v in np.arange(dw[0], dw[1], para.interval_v):
        for w in np.arange(dw[2], dw[3], para.interval_w):
            temp_traj = predict_trajactory(x, [v, w])
            total_cost = cost(temp_traj)
            if min_cost >= total_cost:
                best_traj = temp_traj
                best_u = [v, w]
                min_cost = total_cost

    return best_traj, best_u


if __name__ == '__main__':
    # 以下仅为初始化变量
    para = Parameter()
    x = para.x_init
    u = para.u_init
    trajactory = np.array(x)
    best_traj=np.array((x,x))
    #开始循环
    while True:
        dw = dynamic_window(x,best_traj)
        best_traj, best_u = traj_filter(x, dw)
        x = motion(x, best_u)
        trajactory = np.vstack((trajactory, x))


        #绘图
        plt.cla()
        plt.plot(x[0], x[1], '*r')
        plt.plot(para.ob[:, 0], para.ob[:, 1], 'ok')
        plt.plot(para.goal[0], para.goal[1], 'hb')
        circle = plt.Circle((x[0], x[1]), para.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        angle_dx=para.robot_radius*math.cos(x[2])
        angle_dy=para.robot_radius*math.sin(x[2])
        plt.plot([x[0],x[0]+angle_dx],[x[1], x[1]+angle_dy],'-k')
        plt.plot(best_traj[:,0],best_traj[:,1],'-g')
        plt.axis('equal')
        plt.grid(True)
        plt.pause(0.001)
        dx = para.goal[0] - x[0]
        dy = para.goal[1] - x[1]
        dist_to_goal = math.hypot(dx, dy)
        if dist_to_goal <= para.robot_radius:
            break
    plt.plot(trajactory[:,0],trajactory[:,1],'-r')
    print("Robot Arrived")
    plt.show()
