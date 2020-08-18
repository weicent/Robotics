function dw = dynamic_window(x, trajectory)
global Vmax Vmin Wmax Wmin vv ww dt buffer_clearance
%当前速度变化区间
vl=[x(4)-vv*dt, x(4)+vv*dt, x(5)-ww*dt, x(5)+ww*dt];
vb=sqrt(2*(clearance_to_ob(trajectory)-buffer_clearance)*vv);%最大安全速度，避免碰撞
wb=sqrt(2*(clearance_to_ob(trajectory)-buffer_clearance)*ww);
dw=[max([Vmin,vl(1),-vb]), min([Vmax, vl(2), vb]),...
    max([Wmin, vl(3),-wb]), min([Wmax, vl(4), wb])];
end