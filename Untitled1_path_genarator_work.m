
clear all
close all
clc

tic
N=4;
k=100; % even number only and 

x_tag=15;
y_tag=-5;
theda_tag=-pi/3;
kappa_tag=0;

x_dest=[x_tag y_tag theda_tag kappa_tag]';
arc=sqrt(x_tag^2+y_tag^2);
p=[0 0 0 arc];
x_end = my_endpoint(p, k);

for i=1:500
delta_x=x_dest-x_end;
J_M= My_Jacobian( N, k, p );
iJ_M=inv(J_M);
temp=iJ_M*delta_x;
delta_p=temp';
p=p+delta_p;
x_end = my_endpoint(p, k);
end

delta_x=x_dest-x_end
M=1000;
xic=[0 0 0 0]';
ZZ = my_dynamic( p, xic );

b=p(1)
c=p(2)
d=p(3)
s_f=p(4)
x=zeros(4,1);
% x=[2 ;3 ;-pi ;0]

S=linspace(0,s_f,M);
Ds=s_f/M;
for k=1:M-1
s=S(k);
x(1,k+1)=x(1,k)+cos(x(3,k))*Ds;
x(2,k+1)=x(2,k)+sin(x(3,k))*Ds;
x(3,k+1)=x(3,k)+(b*s+c*s^2+d*s^3)*Ds;
x(4,k+1)=x(4,k)+(b+2*c*s+3*d*s^2)*Ds;
end




figure (1)
hold on
plot(x(1,:),x(2,:),'b');
xlabel('X')
ylabel('Y')
title('x_{target}=15,  y_{target}=-10')



toc


























