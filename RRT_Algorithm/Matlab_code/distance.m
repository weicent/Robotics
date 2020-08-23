function d=distance(Q1,Q2)
x1=Q1(1);y1=Q1(2);z1=Q1(3);
x2=Q2(1);y2=Q2(2);z2=Q2(3);
d=sqrt((x1-x2)^2+(y1-y2)^2+(z1-z2)^2);
end