function status=motion(x, u)
%��������״̬���ṩ���ٶȿ��Ʊ���������һ��״̬
global dt
x(3)=x(3)+u(2)*dt;
x(1)=x(1)+u(1)*dt*cos(x(3));
x(2)=x(2)+u(1)*dt*sin(x(3));
x(4)=u(1);
x(5)=u(2);
status=x;