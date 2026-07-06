function [ feasible ] = my_feasible_test( x_tag, y_tag )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

x_dest=[x_tag y_tag 0 0]';
arc=sqrt(x_tag^2+y_tag^2);

N=4; % Jacobian Matrix rank
k=20; % The simpson integal partition number, only in even number.  
p=[0 0 0 arc];
x_end = my_endpoint(p, k);

for i=1:100
delta_x=x_dest-x_end;
J_M= My_Jacobian( N, k, p );
iJ_M=pinv(J_M);
temp=iJ_M*delta_x;
delta_p=temp';
p=p+delta_p;
x_end = my_endpoint(p, k);
end

delta_x=x_dest-x_end;
ID=delta_x'*delta_x;

if ID<0.5
    feasible=1;
else
    feasible=0;
end




end