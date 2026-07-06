% 航向角計算：依據 dx/dy 的正負判斷所在象限，回傳 0～2π 範圍的絕對航向角


function [ theda Quadrant] = my_yaw_angle( delta )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%%%      Direct Check    %%%
if delta(1)>0 && delta(2)>0
    Quadrant=1;
    theda=atan(delta(2)/delta(1));    
elseif delta(1)<0 && delta(2)>0
    Quadrant=2;
    delta_x=abs(delta(1));
    delta_y=delta(2);
    theda=atan(delta_y/delta_x);
    theda=pi-theda;
elseif delta(1)<0 && delta(2)<0
    Quadrant=3;
    delta_x=abs(delta(1));
    delta_y=abs(delta(2));
    theda=atan(delta_y/delta_x);
    theda=pi+theda;       
elseif delta(1)>0 && delta(2)<0
    Quadrant=4;
    delta_x=abs(delta(1));
    delta_y=abs(delta(2));
    theda=atan(delta_y/delta_x);
    theda=2*pi-theda;     
elseif delta(1)>0 && delta(2)==0
    Quadrant=5;
    theda=0;
elseif delta(1)==0 && delta(2)>0
    Quadrant=6;
    theda=pi/2;
elseif delta(1)<0 && delta(2)==0
    Quadrant=7;
    theda=pi;
elseif delta(1)==0 && delta(2)<0
    Quadrant=8;
    theda=3*pi/2;
else
    Quadrant=6;
end


end

