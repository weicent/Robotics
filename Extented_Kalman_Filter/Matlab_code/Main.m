clc
clear
close all
%% 初始化参数
time=0;
dt=0.1;
simlation_time=20;
%初始化状态矩阵
xActual=zeros(4,1);
hx_actual=xActual;
xEKF=zeros(4,1);
hx_EKF=xEKF;
x_Predict=zeros(4,1);
hx_Predict=x_Predict;
zObserve=zeros(2,1);
hz=zObserve;
pEKF=eye(4);%初始化不确定性协方差矩阵
Q=diag([0.01 0.01 0.01 1]);%过程噪声协方差
v=1;%1m/s
w=0.1;%0.1rad/s
u_Exact=[w; v];%初始化传感器数值,此为精确数值，由于匀速运动因此此项恒定不变
R=diag([0.5,0.5]);
H=[[1 0; 0 1] zeros(2)];
JH=[[1 0; 0 1] zeros(2)];
GPS_Noise=diag([0.1, 0.1]);%x和y的噪音相互独立
Sensor_Noise=diag([0.25, 0.3]);%角速度传感器噪音 速度噪音

%% 运行算法
while time<=simlation_time
    
    time = time+dt;
    
    [xActual, zObserve, uNoise, x_Predict]=...
        IterationUpdating(GPS_Noise, Sensor_Noise,...
        H, u_Exact, xActual, x_Predict, dt);
    
    %计算JF矩阵，即非线性雅可比矩阵
    JF=[1 0 -v*dt*sin(xEKF(3,1)) dt*cos(xEKF(3,1));
        0 1 v*dt*cos(xEKF(3,1)) dt*sin(xEKF(3,1));
        0 0 1 0;
        0 0 0 1];
    
    [xEKF, pEKF]=EKF_Algorithm(xEKF, uNoise, pEKF,...
        zObserve, H, R, Q, JF, JH, dt);
    %存储历史数据
    hx_actual=[hx_actual, xActual];
    hx_Predict=[hx_Predict, x_Predict];
    hx_EKF=[hx_EKF, xEKF];
    hz=[hz, zObserve];
    %绘图
    plot(hz(1,:),hz(2,:),'o','Color','[0.3 0.5 0.4]');
    hold on;
    plot(hx_actual(1,:),hx_actual(2,:),'-b','LineWidth',2);
    plot(hx_Predict(1,:),hx_Predict(2,:),'-k','LineWidth',2);
    plot(hx_EKF(1,:),hx_EKF(2,:),'-r','LineWidth',2);
    
    legend('GPS data','Actual Trace','Predicted Trace',...
        'EKF Trace','Location','northoutside',...
        'Orientation','horizontal');
    legend('boxoff')
    grid on;
    axis equal;
    pause(0.001);
end 