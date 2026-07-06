clear all
close all
clc

now=[1;0];
next=[0;0];

delta=(next-now)';

delta=[1;-1]
r=norm(delta);
 [theda Quad] = my_yaw_angle( delta )
 Angle=rad2deg(theda)