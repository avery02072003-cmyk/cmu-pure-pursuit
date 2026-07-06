% 積分工具組：支援 Jacobian 計算的數值積分模組


function [ output_args1 output_args2 ] = my_Fkn_Gkn(power, part_number, p_ceff)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
p=p_ceff;
n=power;
k=part_number;
L=p(4);
s=linspace(0,L,k);
delta_s=s(2)-s(1);

for i=1:k
    
    
    z=rem(i,2);
    [ f_k(i) g_k(i) ] = my_fk_gk(p,s(i));
    if z==0
    w(i)=4; 
        f_Temp(i)=w(i)*f_k(i)*s(i)^n;
        g_Temp(i)=w(i)*g_k(i)*s(i)^n;
    else
        w(i)=2;
        f_Temp(i)=w(i)*f_k(i)*s(i)^n;
        g_Temp(i)=w(i)*g_k(i)*s(i)^n;
    end
    
    
end


        w(1)=1;
        f_Temp(1)=w(1)*f_k(1)*s(1)^n;
        g_Temp(1)=w(1)*g_k(1)*s(1)^n;
        w(k)=1;
        f_Temp(k)=w(k)*f_k(k)*s(k)^n; 
        g_Temp(k)=w(k)*g_k(k)*s(k)^n;


       f_Temp2=sum(f_Temp);
       g_Temp2=sum(g_Temp);
       output_args1=f_Temp2*(delta_s/3);
       output_args2=g_Temp2*(delta_s/3);

end

