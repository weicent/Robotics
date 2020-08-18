function total_cost=cost(traj)
%评价函数，返回每条路径的cost，越低越好
global Heading_cost Velocity_cost Obstacle_cost Vmax goal robot_radius
robot_start=[traj(1,1) traj(1,2)];
robot_end=[traj(end,1) traj(end,2)];
vec_robot=robot_end-robot_start;
vec_goal=goal-robot_start;
anl_rob=atan2(vec_robot(2), vec_robot(1));
anl_goal=atan2(vec_goal(2), vec_goal(1));
if anl_rob*anl_goal>=0
    angle_diff=abs(anl_goal-anl_rob);
else
    angle_diff=abs(anl_goal)+abs(anl_rob);
    if angle_diff>pi
        angle_diff=2*pi-angle_diff;
    end
end

speed_diff=Vmax-traj(end,4);

dist=clearance_to_ob(traj);
if dist<=robot_radius
    obst_cost=+inf;
else
    obst_cost=1/dist;
end
total_cost=Heading_cost*angle_diff+Velocity_cost*...
    speed_diff+Obstacle_cost*obst_cost;