% 車輛運動學積分：依 κ(s)=b·s+c·s²+d·s³ 展開車輛狀態，對弧長積分 1000 步產生完整路徑軌跡

function [ output_args ] = my_dynamic( p, xic )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
M=1000; % ODE with 1000 sample points
b=p(1);
c=p(2);
d=p(3);
s_f=p(4);
x=zeros(4,M);
x(:,1)=xic;
S=linspace(0,s_f,M);
Ds=s_f/M;
for k=1:M-1
s=S(k);
x(1,k+1)=x(1,k)+cos(x(3,k))*Ds;
x(2,k+1)=x(2,k)+sin(x(3,k))*Ds;
x(3,k+1)=x(3,k)+(b*s+c*s^2+d*s^3)*Ds;
x(4,k+1)=x(4,k)+(b+2*c*s+3*d*s^2)*Ds;
end
output_args=x;
end

