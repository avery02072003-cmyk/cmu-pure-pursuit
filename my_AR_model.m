% GPS 路徑擬合：由 GPS 座標反算曲率多項式係數，用最小二乘法擬合 κ(s)=b·s+c·s²+d·s³


function [ c_eff  S alfa] = my_AR_model( Gps_x,Gps_y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Temp=[Gps_x;Gps_y];
M=length(Gps_x);

for i=2:M    
v2=Temp(:,i);    
v1=Temp(:,i-1);
delta=v2-v1;
alfa(i-1)=atan(delta(2)/delta(1));
end

i=0;
S(1)=0;
for i=2:M-1  
    delta_x=Temp(1,i)-Temp(1,i-1);
    delta_y=Temp(2,i)-Temp(2,i-1);
    delta_S=sqrt(delta_x^2+delta_y^2);
    S(i)=S(i-1)+delta_S;
    Ds=S(i)-S(i-1);    
    y(i-1,1)=(alfa(i)-alfa(i-1))/Ds;
    A(i-1,:)=[S(i-1) S(i-1)^2 S(i-1)^3];    
end

A_t=A'
M=inv(A_t*A)
c_eff=M*A_t*y

end

