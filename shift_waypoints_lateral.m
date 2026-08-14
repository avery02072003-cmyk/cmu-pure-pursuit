% =========================================================================
% 檔案名稱: shift_waypoints_lateral.m
%
% 功能：把一串 waypoints 沿著「路徑法線方向」整體平移距離 d，用來從一條
%       中心線 waypoints 產生「側向偏移 d 公尺」的另一條平行候選路徑。
%       這是「多候選路徑（Multi-Path）」機制的第一步：同一組 waypoints，
%       分別用 d = -lane_width/2 ~ +lane_width/2 平移出 N_paths 條候選路徑，
%       再交給 stitch_local_path.m 逐段擬合成平滑曲線。
%
% 數學原理：
%   對一串離散點 wp=[x_i, y_i]，先用中央差分（MATLAB 內建 gradient()）
%   估計每一點的局部切線方向（航向角）：
%       heading_i = atan2(dy_i, dx_i) ，其中 (dx_i, dy_i) = gradient(x,y)
%
%   路徑的「法線方向（單位向量，指向左手邊）」是切線方向逆時針轉 90 度：
%       tangent = (cos(heading), sin(heading))
%       normal  = (-sin(heading), cos(heading))   ← 切線向量逆時針轉90度
%
%   把每一點沿法線方向平移 d（d>0 代表往左偏移、d<0 代表往右偏移）：
%       x_out_i = x_i + d * (-sin(heading_i))
%       y_out_i = y_i + d *  cos(heading_i)
%
%   這就是「平行曲線（offset curve / parallel curve）」的近似做法：
%   嚴格來說，一條曲線的真平行曲線半徑會隨曲率改變（內側曲率變大、
%   外側曲率變小），但本函式只是「每一點各自獨立沿法線平移固定距離 d」
%   的近似版本，不是精確的包絡線幾何。這個近似在 waypoint 間距夠密、
%   偏移量 d 遠小於路徑曲率半徑時已經足夠準確；本專案 waypoint 間距
%   （my_multi_path 用 ~0.3m、generate_local_paths 用 1.5m）跟偏移量
%   （最大 ±lane_width/2=1.8m）相比，都在合理誤差範圍內。
%
% 輸入：
%   wp : Nx2 矩陣 [x, y]，要被平移的原始 waypoints（中心線）
%   d  : 側向平移量（m），正值往路徑前進方向的左手邊偏移，負值往右
%
% 輸出：
%   wp_out : Nx2 矩陣，平移後的新 waypoints
%
% 呼叫端：
%   - my_multi_path.m（整條路線一次性產生候選路徑，供 STEP2 預覽）
%   - generate_local_paths.m（即時局部候選路徑生成，每次 replan 呼叫）
% =========================================================================

function wp_out = shift_waypoints_lateral(wp, d)
    dx = gradient(wp(:,1));
    dy = gradient(wp(:,2));
    heading = atan2(dy, dx);      % 每一點的局部切線方向
    nx = -sin(heading);           % 法線方向單位向量 x 分量（切線逆時針轉90度）
    ny = cos(heading);            % 法線方向單位向量 y 分量
    wp_out = [wp(:,1) + d*nx, wp(:,2) + d*ny];   % 沿法線方向平移 d
end
