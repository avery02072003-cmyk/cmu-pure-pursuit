% =========================================================================
% 檔案名稱: my_yaw_angle.m
%
% 功能：給定一個位移向量 delta=[dx,dy]，用「逐象限判斷 + atan」的方式
%       手動算出這個向量的絕對航向角，回傳範圍是 [0, 2*pi)（而不是
%       MATLAB 內建 atan2 回傳的 (-pi, pi]）。
%
%       這其實等價於 mod(atan2(delta(2), delta(1)), 2*pi) 這一行內建
%       函式呼叫就能做到的事，只是用最原始的「判斷落在第幾象限、
%       分開處理」寫法實現（歷史寫法，供 Untitled4_tracking_trajectory_test.m
%       這個母路徑產生腳本呼叫，判斷用哪個角度旋轉座標系）。Quadrant
%       回傳值標示落在第幾象限（1~4 為標準象限，5~8 為落在座標軸上的
%       特殊情況），呼叫端目前沒有實際使用這個回傳值。
%
% 輸入：
%   delta : 2x1（或1x2）位移向量 [dx, dy]
%
% 輸出：
%   theda    : 絕對航向角 [0, 2*pi) rad
%   Quadrant : 落在第幾象限（1~4 標準象限，5~8 為落在座標軸上）
% =========================================================================

function [ theda Quadrant] = my_yaw_angle( delta )
%%%      依 dx, dy 正負號逐一判斷象限    %%%
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

