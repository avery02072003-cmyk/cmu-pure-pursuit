% 場景測試腳本：S 曲線 下的路徑生成驗證


clear all
close all
clc


points=360*3;
t=linspace(pi,0,points);
a=15;
for i=1:points
x1(i)=(a+5)+a*cos(t(i));
y1(i)=(a+5)+a*sin(t(i));
end

x2=35*ones(1,360);
y2=linspace(19.95,-10,360);

t=linspace(-pi,0,points);
for i=1:points
x3(i)=55+20*cos(t(i));
y3(i)=-10+20*sin(t(i));
end

x0=5*ones(1,360);
y0=linspace(-24.95,19.95,360);

xe=75*ones(1,360);
ye=linspace(-9.95,30,360);

points=360*3;
t=linspace(-pi,0,points);
a=15;
for i=1:points
x5(i)=(a+5)+a*cos(t(i));
y5(i)=-25+a*sin(t(i));
end


x_=[x0 x1 x2 x3 xe] ; 
y_=[y0 y1 y2 y3 ye];
M=length(x_);


clear x0 x1 x2 x3 xe
clear y0 y1 y2 y3 ye
q=2
x(1)=x_(M)
y(1)=y_(M)
for i=1:M-1
    x(q)=x_(M-i);
    y(q)=y_(M-i);
    q=q+1;
end

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

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:20

start=(k-1)*150+2
target=150*k
if target>=M
target=M-1;
end


now=[x(start);y(start)];
next=[x(target);y(target)];

delta=next-now
theda_in=theda(start)
theda_out=theda(target)

DT=theda_out-theda_in;
if DT<-pi
theda_out=theda(target)+2*pi
elseif DT>pi
    theda_in=theda(start)+2*pi;
else
end

phi_tag=theda_out-theda_in
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
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%   Path Rotation   %%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
xlabel('X')
ylabel('Y')
plot(ZZ_R(1,:),ZZ_R(2,:),'b');
title('Path Cruve Fitting Demo')
axis equal
pause(0.5)
end






