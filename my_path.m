% 路徑規劃核心：Newton-Raphson 迭代求解曲率多項式係數 p，使終點滿足目標位姿 (x,y,φ,0)，疊代 500 次


function [ Output  ceff] = my_path( x_tag, y_tag, phi_tag, xic )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

x_dest=[x_tag y_tag phi_tag 0]';
arc=sqrt(x_tag^2+y_tag^2);

N=4; % Jacobian Matrix rank
k=20; % The simpson integal partition number, only in even number.  
p=[0 0 0 arc];
x_end = my_endpoint(p, k);

for i=1:500
delta_x=x_dest-x_end;
J_M= My_Jacobian( N, k, p );
iJ_M=pinv(J_M);
temp=iJ_M*delta_x;
delta_p=temp';
p=p+delta_p;
x_end = my_endpoint(p, k);
end

delta_x=x_dest-x_end;
Output= my_dynamic( p, xic );
ceff=p;
end

