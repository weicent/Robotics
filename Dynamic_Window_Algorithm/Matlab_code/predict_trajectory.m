function traj=predict_trajectory(x, u)
%返回一条预测的轨迹
global dt predict_T
time=0;
traj=[];
while time<=predict_T
    time=time+dt;
    x=motion(x,u);
    traj=[traj; x];
end