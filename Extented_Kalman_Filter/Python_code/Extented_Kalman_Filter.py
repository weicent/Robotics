import numpy as np
import math
import matplotlib.pyplot as plt

'''此示例展示了2D平面上机器人的航迹EKF优化估算，参数包含[x y 角度 速度]'''
'''声明全局变量、初始化参数'''
# 过程噪声协方差，若为零（即不考虑Q）则KMF估算值随着迭代的增加会比实际值偏差很多
Q = np.diag([0.1, 0.1, np.deg2rad(1), 1])**2
# 测量噪音，此处为np.diag()方法生成2*2矩阵，方便后续乘以随
# 机数2*1的矩阵（因为速度角度传感器是分开独立单元，因此随机
# 性并不相关，需要用2*1的矩阵分别生成两个随机数），对于GPS
# xy方向上的湍流也都如此（即认为随机性不相关）
GPS_NOISE = np.diag([0.5, 0.5])**2
SENSOR_NOISE = np.diag([1, np.deg2rad(30)]) **2 # 传感器噪音[速度，角度]
# 传感器精度误差
R = np.diag([1, 1])**2
# 初始化真实状态、预测状态、EKF估计状态、不确定性
xActual = np.zeros((4, 1))
xPredict = np.zeros((4, 1))
xEKF = np.zeros((4, 1))
pEKF = np.eye(4)
# 初始化历史数据存储矩阵
hxActual = np.zeros((4, 1))
hxPredict = np.zeros((4, 1))
hxEKF = np.zeros((4, 1))
hzObserved = np.zeros((2, 1))
# 时间步长
t = 0
DT = 0.1
TOTAL_TIME = 50
# 初始化速度与角速度
VELOCITY = 1
ANGLE = 0.1  # rad/s角速度
uExact = np.array([[VELOCITY],
                   [ANGLE]])
# 测量矩阵
H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])
# JacobianH，与z=Hx有关，主要看z的值是什么，此处为x，y（即GPS观测值）因此对方程组以x和y进行偏微分
JACOBIAN_H = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0]])

'''EKF算法实现'''


def Predict(xActual, xPredict, uExact):
    '''生成真实值，预测值，观测值'''
    xActual = x_current_from_precede(xActual, uExact)  # 此为飞行器实际状态
    u_Noise = uExact + SENSOR_NOISE @ np.random.randn(2, 1)  # 传感器读数（即实际值+噪音）
    zObserved = H @ xActual + GPS_NOISE @ np.random.randn(2, 1)  # 观测数据=实际值（x，y）+GPS误差扰动
    xPredict = x_current_from_precede(xPredict, u_Noise)

    return xActual, u_Noise, zObserved, xPredict


def x_current_from_precede(x, u):
    F = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 0]])
    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0, DT],
                  [1, 0]])
    xA = F @ x + B @ u
    return xA


def jacobian_F(x, u):
    v = x[3, 0]
    w = x[2, 0]
    JF = np.array([[1, 0, -v * DT * math.sin(w), DT * math.cos(w)],
                   [0, 1, v * DT * math.cos(w), DT * math.sin(w)],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    return JF


def extented_kalman_filter(x, p, z, u):
    xPredict = x_current_from_precede(x, u)
    JF = jacobian_F(xEKF, u)
    pPredict = JF @ p @ JF.T + Q
    zPred = H @ xPredict
    y = z - zPred
    S = JACOBIAN_H @ pPredict @ JACOBIAN_H.T + R
    K = pPredict @ JACOBIAN_H.T @ np.linalg.inv(S)
    xEKF_Current = xPredict + K @ y
    p_Current = (np.eye(4) - K @ JACOBIAN_H) @ pPredict
    return xEKF_Current, p_Current


# 调用方法，启用卡曼尔滤波器
while t <= TOTAL_TIME:
    t += DT
    xActual, u_Noise, zObserved, xPredict = Predict(xActual, xPredict, uExact)
    xEKF, pEKF = extented_kalman_filter(xEKF, pEKF, zObserved, u_Noise)
    # 存储历史数据，以便后续动态绘图
    hxEKF = np.hstack((hxEKF, xEKF))
    hxActual = np.hstack((hxActual, xActual))
    hxPredict = np.hstack((hxPredict, xPredict))
    hzObserved = np.hstack((hzObserved, zObserved))
    # 绘制航迹
    plt.cla()
    plt.plot(hzObserved[0, :].flatten(), hzObserved[1, :].flatten(), '.g', label='GPS')
    plt.plot(hxActual[0, :].flatten(), hxActual[1, :].flatten(), '-b', label='Actual Trace')
    plt.plot(hxPredict[0, :].flatten(), hxPredict[1, :].flatten(), '-k', label='Predicted')
    plt.plot(hxEKF[0, :].flatten(), hxEKF[1, :].flatten(), '-r', label='EKF')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Extented Kalman Filter')
    plt.legend()
    plt.pause(0.001)
plt.show()
