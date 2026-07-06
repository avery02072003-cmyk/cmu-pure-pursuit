% 速度曲線生成：以三次多項式擬合起點與終點的速度/加速度邊界條件，生成 1000 點速度剖面


function [ output1 output2 ] = my_velocity( Q1,Q2 )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
s0=0; v0=Q1(2); a0=Q1(3);
s1=Q2(1); v1=Q2(2); a1=Q2(3);

q(1)=v0;
q(2)=a0;
q(3)=(-2*a0/s1)-(a1/s1)-(3*v0/s1^2)+3*v1/s1^2;
q(4)=(a0/s1^2)+(a1/s1^2)+(2*v0/s1^3)-(2*v1/s1^3);

s=linspace(0,s1,1000);

    for j=1:1000 
        velocity(j)=q(1)+q(2)*s(j)+q(3)*s(j)^2+q(4)*s(j)^3;
    end
    
    output1=velocity;
    output2=q;

end

