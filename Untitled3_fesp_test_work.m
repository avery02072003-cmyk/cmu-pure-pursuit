clear all
close all
clc
tic
x_tag=15; y_tag=6;phi_tag=pi/2;
xic=[0 0 0 0]';
[ ZZ ] = my_path( x_tag, y_tag, phi_tag, xic );

figure (1)
plot(ZZ(1,:),ZZ(2,:),'r');
xlabel('X')
ylabel('Y')

figure (2)
plot(ZZ(3,:));

toc




