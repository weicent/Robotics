function [best_traj, best_u]=traj_filter(x, dw)
%根据动态窗口生成一系列路径，并从中返回最优路径策略
%但需要注意的是动态窗口法只能返回局部最优解
global interval_v interval_w
min_cost=+inf;
for v=dw(1):interval_v:dw(2)
    for w=dw(3):interval_w:dw(4)
        u=[v, w];
        traj=predict_trajectory(x, u);
        total_cost=cost(traj);
        if min_cost>=total_cost
            min_cost=total_cost;
            best_u=u;
            best_traj=traj;
        end
    end
end