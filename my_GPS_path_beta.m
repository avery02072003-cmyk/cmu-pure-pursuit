% GPS 路徑生成封裝：輸入目標位姿+旋轉角，呼叫 my_path 後做座標旋轉，回傳全域路徑點


function [ output_args ] = my_GPS_path_beta( x_tag, y_tag, theda_tag, rotation )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

alfa=rotation;
TFS=[cos(alfa) -sin(alfa); sin(alfa) cos(alfa)];
xic=[0 0 0 0]';
ZZ = my_path( x_tag, y_tag, theda_tag, xic );

for i=1:1000
    Temp=[ ZZ(1,i) ZZ(2,i)]';    
    ZZ_R(:,i)=TFS*Temp;
end
   output_args=ZZ_R;
end

