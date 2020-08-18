function [best_traj, best_u]=traj_filter(x, dw)
%���ݶ�̬��������һϵ��·���������з�������·������
%����Ҫע����Ƕ�̬���ڷ�ֻ�ܷ��ؾֲ����Ž�
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