clc
clear
close all
%% 初始化参数

global Vmax Vmin Wmax Wmin vv ww interval_v interval_w...
    dt predict_T Heading_cost Velocity_cost Obstacle_cost...
    ob robot_radius buffer_clearance goal
x_init = [0, 0, 0, 0, 0];
u_init = [0, 0];
Vmax = 1;
Vmin = -0.5;
Wmax = 40 * pi / 180;  % 角速度
Wmin = -40 * pi / 180;
vv = 0.2;  % m/s**2
ww = 45 * pi / 180;  % rad/s**2
interval_v = 0.01;  % 动态取值分辨率
interval_w = 0.1 * pi / 180;
dt = 0.1;  % 时间步长
predict_T = 3;  % 窗口预测时间长度
Heading_cost = 0.07;  % 评价函数系数，用作校正权重，分别为角度系数，速度系数，障碍距离系数
Velocity_cost = 1.2;
Obstacle_cost = 2;
ob = [-1, -1; 0, 2; 2, 6; 2, 8; 3, 9.27; 3.79, 9.39; 7.25, 8.97;...
    7.0, 2.0; 3.0, 4.0; 6.0, 5.0; 3.5, 5.8; 6.0, 9.0; 8.8, 9.0;...
    5.0, 9.0; 7.5, 3.0; 9.0, 8.0; 5.8, 4.4; 12.0, 12.0; 3.0, 2.0;...
    13.0, 13.0];
robot_radius = 1;
buffer_clearance = 1.2;  % 与障碍物之间的缓冲距离
goal = [5, 7];
maxpower=0.7;
%% 主体循环
x=x_init;
u=u_init;
traj=x;
best_traj=[x;x];
while true
    dw=dynamic_window(x,best_traj);
    [best_traj, best_u]=traj_filter(x, dw);
    if best_u(1)<0.1
        if abs(best_u(2))<0.09 || abs(best_u(2)-pi)<0.09
            best_u=[best_u(1) best_u(2)+ww*maxpower];
        end
    end          
    x=motion(x, best_u);
    traj=[traj; x];
    
    plot(x(1),x(2),'*r');
    hold on
    plot(ob(:,1),ob(:,2),'ok');
    plot(goal(1),goal(2),'hr');
    plot(best_traj(:,1),best_traj(:,2),'-g');
    plot([x(1), x(1)+robot_radius*cos(x(3))],...
        [x(2), x(2)+robot_radius*sin(x(3))],'-k');
    viscircles([x(1), x(2)],robot_radius,'color','r');
    hold off
    axis equal;
    grid on
    pause(0.0001)
    dx=goal(1)-x(1);
    dy=goal(2)-x(2);
    distogoal=hypot(dx,dy);
    if distogoal<robot_radius
        break
    end
end
hold on;
plot(traj(:,1),traj(:,2),'-b');
fprintf('Robot Arrived')
    
    