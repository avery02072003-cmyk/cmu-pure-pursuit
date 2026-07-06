% 積分工具組：支援 Jacobian 計算的數值積分模組


function [ output_args_a output_args_b ] = my_fk_gk( p_ceff, s_now )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
p=p_ceff;
s=s_now;
theda=(1/2)*p(1)*s^2+(1/3)*p(2)*s^3+(1/4)*p(3)*s^4;
output_args_a = cos(theda);
output_args_b = sin(theda);

end

