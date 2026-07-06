% 場景測試腳本：逆時針圓 下的路徑生成驗證


clear all
close all
clc

%%%%%%%%%   �f�ɰw ���έy�D %%%%%%%%%%%%%%%

sample=360*7;

t=linspace(0,2*pi,sample);
a=35;

for i=1:sample
x(i)=(a+5)+a*cos(t(i));
y(i)=(a+5)+a*sin(t(i));
end

Temp0=[x;y];
M=length(x);

%%%%%~~~~~~~~~~~~~~~~~~~~~~~  Get Heading Angle ~~~~~~~~~~~~~~~~~%%%%%%%%%%

for i=1:M-1
y_d=y(i+1)-y(i);
x_d=x(i+1)-x(i);
theda(i)=atan2(y_d,x_d);
theda_deg(i)=rad2deg(theda(i));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure (1)
% plot(theda);
%%%%%%%%%%%%%%%%%%%%%%%       Path Generator     %%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:13
    
start=2000; 
start=(k-1)*200+2
target=200*k
if target>=M
target=M-1;
end

now=[x(start);y(start)];
next=[x(target);y(target)];
delta=next-now
theda_in=theda(start)
theda_out=theda(target)
if theda_in>0 && theda_out<0
    theda_in=theda(start)-2*pi;
else
%     theda_in=theda(start);
end

phi_tag=theda(target)-theda_in
check=rad2deg(phi_tag)

delta_b=now-[x(start-1);y(start-1)]
[ alfa Quadrant] = my_yaw_angle( delta_b )
TFS=[cos(alfa) -sin(alfa); sin(alfa) cos(alfa)];  

iTFS=inv(TFS)
QQQ=iTFS*delta

x_tag=QQQ(1);
y_tag=QQQ(2);

xic=[0 0 0 0]';
[ ZZ p] = my_path( x_tag, y_tag, phi_tag, xic );



%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Path Rotation   %%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:1000
Temp=[ ZZ(1,i) ZZ(2,i)]';    
ZZ_R(:,i)=TFS*Temp+now;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure (2)
hold on
plot(x,y,'r');


plot(next(1,1),next(2,1),'k*');
plot(now(1,1),now(2,1),'ks');
axis equal
xlabel('X')
ylabel('Y')
plot(ZZ_R(1,:),ZZ_R(2,:),'b');
title('Path Cruve Fitting Demo')
pause(0.5)
end

