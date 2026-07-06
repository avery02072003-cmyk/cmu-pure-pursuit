clear all
close all
clc


N=4;

x_tag=15;
y_tag=linspace(-5,5,5);
theda_tag=pi/6;
xic=[0 0 0 0]';

alfa=pi/2
TFS_X=[cos(alfa) -sin(alfa);sin(alfa) cos(alfa)];


TFS(:,:,1)=[1 0;0 1];    %%  direct East
TFS(:,:,2)=[0 1;1 0];    %%  direct North
TFS(:,:,3)=[-1 0;0 1];   %%  direct West
TFS(:,:,4)=[0 1; -1 0];  %%  direct south




for k=1:5

ZZ(:,:,k) = my_path( x_tag, y_tag(k), theda_tag, xic );

for i=1:1000
Temp=[ ZZ(1,i,k) ZZ(2,i,k)]';    
ZZ_R(:,i,k)=TFS(:,:,2)*Temp;
ZZ_R(:,i,k)=TFS_X*Temp;

end

figure (1)
hold on
plot(ZZ(1,:,k),ZZ(2,:,k),'b');
xlabel('X')
ylabel('Y')

figure (2)
hold on
plot(ZZ_R(1,:,k),ZZ_R(2,:,k),'b');
xlabel('X')
ylabel('Y')

end




