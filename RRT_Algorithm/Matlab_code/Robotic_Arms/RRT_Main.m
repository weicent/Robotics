%% initialization
clc
clear
close all
tic;
%% Obstacle
c=[0 700; 575 575; -550 550; 700 350];
r=[50, 50, 65, 10];

%% Robot Pose: initial and final
Qi=[20 0 0];% initial position
Qg=[160 0.0001 0.0001];% final position
l1=500;l2=300;l3=100;l4=50; %length of 3 arms

%% RRT Variables
%step
step=35; % step size
t_step=1/step; % time step
% RRT variables
Qnear=[50 50 50]; % initializing Qnear
Qnew=[50 50 50]; % initializing Qnew
parent=[1]; % initializing array to hold the index of parent to each node.Parent of Qi is Qi itself.
near_node=1; % initializing index of Qnear
found=0; % initializing terminating flag
d=1000; %initialise distance
tmax=0;
edge=[0];
% w1=3; %weight factor for first arm
% w2=2; %weight factor for second arm
% w3=1; %weight factor for third arm
N=1e10; % max iteration index
G=[];
G=[G;Qi]; %initialize G

%% start RRT

for i=1:N
    % coin toss, generate Qrand or Qg
    coin=rand(1)*100;
    if coin<=99
        t1=rand(1)*180;t2=rand(1)*180-90;t3= rand(1)*180-90;
        Qrand=[t1 t2 t3];%*rad; %theta1 from 0 to 180, theta2,3 from -90 to 90
    else
        Qrand=Qg;
    end
    % find closest point in G to Qrand
    d=1000;
    [row, column]=size(G);
    Qnear=G(1,1:column);
    for j=1:row
        rho=distance(G(j,:),Qrand);
        if rho<d
            Qnear=G(j,:);
            d=rho;
            near_node=j;
        end
    end
    
    t=0;
    tmax=0;
    while t<1
        
        Qtemp=(1-t)*Qnear+t*Qrand;
        %line segment of arms, the end of 1st arm is orignial point
        po=position(Qtemp,l1,l2,l3,l4);
        A=[0 0];B=po(1,:); %segements of 1st arm is A B
        C=po(2,:); % segements end point of second arm is B C
        D=po(3,:);
        % end effector coordinates
        E=po(4,:);
        F=po(5,:);
        % calculate the collision
        p=length(r); %length of r is equal to number of row of c
        for iok=1:p
            Center=c(iok,:); % center of iok-th circle
            Radius=r(iok); %radius of iok-th circle
            delta(iok,1)=circle_line_intersection(A, B, Center, Radius); %first arm
            delta(iok,2)=circle_line_intersection(B, C, Center, Radius); %second arm
            delta(iok,3)=circle_line_intersection(C, D, Center, Radius); %third arm
            delta(iok,4)=circle_line_intersection(E, F, Center, Radius); %end effector
            
        end
        
        
        if (sum(sum(delta))==0) %if no collision/intersection
            Qnew=Qtemp;
            tmax=t;
        else
            break;
        end
        t=t+t_step;
    end
    % if exist a position between Qnear and Qrand that not cllide
    
    if (tmax~=0)
        if (tmax==1-t_step)
            G=[G;Qrand];
            parent=[parent;near_node];
            edge=[edge;1];
        else
            G=[G;Qnew];
            parent=[parent;near_node];
            edge=[edge;tmax];
        end
        if Qnew==Qg
            found=1;
            break;
        end
    end
    
end

%% Plot the motion
% draw obstacle
[n m]=size(c);
theta=0:pi/20:2*pi;
for i=1:n
    x=r(i)*cos(theta)+c(i,1);
    y=r(i)*sin(theta)+c(i,2);
    plot(x,y,'-');
    fill(x,y,'r');
    axis equal
    hold on
end
% drawRobots

if found==1
    t=near_node;
    k=parent(t);
    while t~=1
        PT=position(G(t,:),l1,l2,l3,l4);
        drawTrace([0,0],PT(1,:),PT(2,:),PT(3,:),PT(4,:),PT(5,:),'g');
        ttmax=edge(t);
        
        while ttmax>=0
            Qtt=[(1-ttmax)*G(k,1)+ttmax*G(t,1) (1-ttmax)*G(k,2)+ttmax*G(t,2) (1-ttmax)*G(k,3)+ttmax*G(t,3)];
            PT=position(Qtt,l1,l2,l3,l4);
            drawTrace([0,0],PT(1,:),PT(2,:),PT(3,:),PT(4,:),PT(5,:),'g');
            ttmax=ttmax-t_step;
        end
        PT=position(G(k,:),l1,l2,l3,l4);
        drawTrace([0,0],PT(1,:),PT(2,:),PT(3,:),PT(4,:),PT(5,:),'g');
        pause(0.1);
        t=k;
        k=parent(t);
    end
    PT=position(Qi,l1,l2,l3,l4);
    drawTrace([0,0],PT(1,:),PT(2,:),PT(3,:),PT(4,:),PT(5,:),'r');
    PT=position(Qg,l1,l2,l3,l4);
    drawTrace([0,0],PT(1,:),PT(2,:),PT(3,:),PT(4,:),PT(5,:),'r');
end
%test
% figure;
% [n m]=size(c);
% theta=0:pi/20:2*pi;
% for i=1:n
%     x=r(i)*cos(theta)+c(i,1);
%     y=r(i)*sin(theta)+c(i,2);
%     plot(x,y,'-');
%     fill(x,y,'r');
%     hold on
% end
% for i=1:length(G)
%     PT=position(G(i,:),l1,l2,l3,l4);
%     drawTrace([0,0],PT(1,:),PT(2,:),PT(3,:),PT(4,:),PT(5,:));
% end
toc



