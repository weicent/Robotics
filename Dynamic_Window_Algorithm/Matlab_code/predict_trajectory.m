function traj=predict_trajectory(x, u)
%����һ��Ԥ��Ĺ켣
global dt predict_T
time=0;
traj=[];
while time<=predict_T
    time=time+dt;
    x=motion(x,u);
    traj=[traj; x];
end