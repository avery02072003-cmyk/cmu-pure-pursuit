% =========================================================================
% 檔案名稱: shift_waypoints_lateral_profile.m
%
% 功能：shift_waypoints_lateral.m 的姊妹函式（不修改原檔），差別只在
%       側向偏移量從「單一常數 d」改成「逐點對應的向量 d_profile」。
%       shift_waypoints_lateral.m 其實就是本函式在 d_profile 為常數向量
%       時的特例；但因為現有候選路徑產生流程（generate_local_paths.m、
%       my_multi_path.m）已經驗證過、也只需要常數偏移，沒有必要把它們
%       改成呼叫這個更通用的版本——保留兩個檔案，各自對應各自的使用情境，
%       維持「已驗證的東西不要動」原則。
%
% 用途：換道分支的側向偏移不是常數（從目前車道偏移量平滑過渡到目標
%       車道偏移量，見 lane_change_offset_profile.m），需要逐點給定
%       不同的偏移量，本函式就是為此而生。
%
% 數學原理跟 shift_waypoints_lateral.m 完全相同（沿路徑法線方向平移），
% 只是這裡的「平移距離」對每一點都不一樣：
%       heading_i = atan2(dy_i, dx_i)         ← gradient() 估計局部切線方向
%       normal_i  = (-sin(heading_i), cos(heading_i))
%       wp_out_i  = wp_i + d_profile_i * normal_i
%
% 輸入：
%   wp        : Nx2 矩陣 [x, y]，要被平移的原始 waypoints
%   d_profile : Nx1（或與 wp 列數相同）向量，每一點各自的側向平移量，
%               正值往路徑前進方向的左手邊偏移，負值往右
%               （通常由 lane_change_offset_profile.m 算出）
%
% 輸出：
%   wp_out : Nx2 矩陣，平移後的新 waypoints
% =========================================================================

function wp_out = shift_waypoints_lateral_profile(wp, d_profile)
    dx = gradient(wp(:,1));
    dy = gradient(wp(:,2));
    heading = atan2(dy, dx);      % 每一點的局部切線方向
    nx = -sin(heading);           % 法線方向單位向量 x 分量
    ny = cos(heading);            % 法線方向單位向量 y 分量

    d_profile = d_profile(:);     % 統一成列向量，跟 wp 的列對應
    wp_out = [wp(:,1) + d_profile.*nx, wp(:,2) + d_profile.*ny];
end
