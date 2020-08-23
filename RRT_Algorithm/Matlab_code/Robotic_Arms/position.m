function f=position(Qtemp,l1,l2,l3,l4)
f=zeros(5,2);
f(1,:)=[l1*cosd(Qtemp(1)), l1*sind(Qtemp(1))];% Point B
f(2,:)=[l2*cosd(Qtemp(1)+Qtemp(2)), l2*sind(Qtemp(1)+Qtemp(2))]+f(1,:); %C
f(3,:)=[l3*cosd(Qtemp(1)+Qtemp(2)+Qtemp(3)), l3*sind(Qtemp(1)+Qtemp(2)+Qtemp(3))]+f(2,:);%D 
f(4,:)=[0.5*l4*cosd(Qtemp(1)+Qtemp(2)+Qtemp(3)+90), 0.5*l4*sind(Qtemp(1)+Qtemp(2)+Qtemp(3)+90)]+f(3,:); %E
f(5,:)=[0.5*l4*cosd(Qtemp(1)+Qtemp(2)+Qtemp(3)-90), 0.5*l4*sind(Qtemp(1)+Qtemp(2)+Qtemp(3)-90)]+f(3,:); %F
end