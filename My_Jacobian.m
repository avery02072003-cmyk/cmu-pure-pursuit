% 	Jacobian 矩陣：計算終點狀態對係數 p 的偏導數 4×4 矩陣，供 Newton-Raphson 迭代修正


function [ output_args ] = My_Jacobian( col_num, part_num, p_ceff )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
n=col_num;
k=part_num;

p=p_ceff;
s_f=p(4);
s=linspace(0,s_f,k);

for i=1:n
 [ Fk(i) Gk(i)] = my_Fkn_Gkn(i, k, p);
end

J_M=zeros(4,n); 

for i=1:n-1
z=i+1;    
Jacobian_M(1,i)=(-1/z)*Gk(z);
Jacobian_M(2,i)=(1/z)*Fk(z);
Jacobian_M(3,i)=p(4)^(z)/z;
Jacobian_M(4,i)=p(4)^(i);
z=0;
end

theda_sf=(1/2)*p(1)*p(4)^2+(1/3)*p(2)*p(4)^3+(1/4)*p(3)*p(4)^4;

Jacobian_M(1,n)=cos(theda_sf);
Jacobian_M(2,n)=sin(theda_sf);
Jacobian_M(3,n)=p(1)*p(4)+p(2)*p(4)^2+p(3)*p(4)^3;
Jacobian_M(4,n)=p(1)+2*p(2)*p(4)+3*p(3)*p(4)^2;

JJJ=Jacobian_M;

output_args=JJJ;

end

