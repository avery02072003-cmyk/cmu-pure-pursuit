% 主測試腳本：定義橢圓/直線/半圓組成的道路
% 分 segment 逐段呼叫 my_path 並做旋轉拼合，繪製 Tractor-Trailer 追蹤結果


clear all
close all
clc

%%%%%%%%%   �f�ɰw �޳��y�D  %%%%%%%%%%%%%%%

points=360*3;
t=linspace(0,pi,points);
a=15;
for i=1:points
x1(i)=(a+5)+a*cos(t(i));
y1(i)=(a+5)+a*sin(t(i));
end

x2=35*ones(1,360*2);
y2=linspace(-24.95,19.95,360*2);

t=linspace(-pi,0,points);
for i=1:points
x3(i)=55+20*cos(t(i));
y3(i)=-10+20*sin(t(i));
end

x0=5*ones(1,360*2);
y0=linspace(19.96,-24.98,360*2);


points=360*3;
t=linspace(-pi,0,points);
a=15;
for i=1:points
x5(i)=(a+5)+a*cos(t(i));
y5(i)=-25+a*sin(t(i));
end


figure (3)
hold on
plot(x0,y0,'b');
plot(x0(1),y0(1),'bo')
plot(x1,y1,'r');
plot(x1(1),y1(1),'ro')
plot(x2,y2,'g');
plot(x2(1),y2(1),'go')
plot(x5,y5,'k');
plot(x5(1),y5(1),'ko')
axis equal

x=[x2 x1 x0 x5] ; 
y=[y2 y1 y0 y5] ;
M=length(x);

ref_x = [];
ref_y = [];


clear x0 x1 x2 x3 xe
clear y0 y1 y2 y3 ye


%%%%%~~~~~~~~~~~~~~~~~~~~~~~  Get Heading Angle ~~~~~~~~~~~~~~~~~%%%%%%%%%%

for i=1:M-1
y_d=y(i+1)-y(i);
x_d=x(i+1)-x(i);
theda(i)=atan2(y_d,x_d);
theda_deg(i)=rad2deg(theda(i));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure (1)
plot(theda);

%%%%%%%%%%%%%%%%%%%%%%%       Path Generator     %%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:100
    
% start=2000; 
start=(k-1)*120+2
target=120*k
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

ref_x = [ref_x, ZZ_R(1,:)];
ref_y = [ref_y, ZZ_R(2,:)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure (2)
hold on

p1=plot(next(1,1),next(2,1),'ko');
set(gca,'FontSize',20);
set(p1, 'linewidth', 2);
p2=plot(now(1,1),now(2,1),'ks');
set(p2, 'linewidth', 2);
axis equal
xlabel('X (meter)','FontSize',18)
ylabel('Y (meter)','FontSize',18)
p3=plot(x,y,'r');
set(p3, 'linewidth', 2);
p4=plot(ZZ_R(1,:),ZZ_R(2,:),'b');
set(p4, 'linewidth', 2);
title('Tractor-Trailer Tacking Trajectory','FontSize',18 )
legend([p1 p2 p3 p4],{'Tractor Waypoint','Trailer Waypoint','Tractor Trajectory','Trailer Trajectory'},'FontSize',18)
% pause
end

dx = diff(ref_x);
dy = diff(ref_y);
ref_phi = atan2(dy, dx);
ref_phi = [ref_phi, ref_phi(end)];

refpath.x = ref_x(:);
refpath.y = ref_y(:);
refpath.phi = ref_phi(:);
refpath.v = 3.0 * ones(length(ref_x),1);   % 第一版先固定速度
save('reference_path.mat', 'refpath');
