function r=clearance_to_ob(trajectory)
%��������ϰ������
global ob
dx=[];
dy=[];
for obx=ob(:,1).'
    dx=[dx; (trajectory(:,1)-obx).'];
end
for oby=ob(:,2).'
    dy=[dy; (trajectory(:,2)-oby).'];
end
r=min(min(hypot(dx,dy)));