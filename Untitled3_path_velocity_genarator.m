clear all
close all
clc

x_tag=5;
y_tag=linspace(-3,3,5);
theda_tag=0;
xic=[0 0 0 0]';

for k=1:5
[path(:,:,k) p(k,:)]= my_path( x_tag, y_tag(k), theda_tag, xic );
end


%%  Velocity Ganarator %%

v_low=0;v_high=10;N_speed=5;
delta_v=(v_high-v_low)/(N_speed-1);

for k=1:N_speed
v0=v_low;    
v_s(k)=v0+delta_v*(k-1);
v1=v_s(k);
a0=0;a1=0;s0=0;s1=p(k,4);
q(1)=v0;
q(2)=a0;
q(3)=(-2*a0/s1)-(a1/s1)-(3*v0/s1^2)+3*v1/s1^2;
q(4)=(a0/s1^2)+(a1/s1^2)+(2*v0/s1^3)-(2*v1/s1^3);
rho(k,:)=q;
end

for i=1:N_speed
    s=linspace(0,p(i,4),1000);
    for j=1:1000 
        v(i,j)=rho(i,1)+rho(i,2)*s(j)+rho(i,3)*s(j)^2+rho(i,4)*s(j)^3;
    end

    figure (1)
    hold on
    plot(v(i,:),'b');
    
    
end
