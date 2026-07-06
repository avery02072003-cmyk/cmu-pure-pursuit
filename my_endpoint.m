% 終點狀態計算：由係數 p 算出路徑終點的 (x,y,θ,κ)，供迭代收斂使用


function [ output_args ] = my_endpoint( p_ceff, sample_num  )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

p = p_ceff;

s= p(4);

sample=sample_num ;

[ x_end y_end ] = my_Fkn_Gkn(0, sample, p);

kappa=p(1)*s+p(2)*s^2+p(3)*s^3;

theda=(1/2)*p(1)*s^2+(1/3)*p(2)*s^3+(1/4)*p(3)*s^4;

output_args=[x_end;y_end;theda;kappa];

end

