% shift_waypoints_lateral.m — 沿路徑法線方向橫向平移一組 waypoints
% 抽出自 my_multi_path.m，供整條路線預覽 (my_multi_path) 與即時局部生成 (generate_local_paths) 共用

function wp_out = shift_waypoints_lateral(wp, d)
    dx = gradient(wp(:,1));
    dy = gradient(wp(:,2));
    heading = atan2(dy, dx);
    nx = -sin(heading);
    ny = cos(heading);
    wp_out = [wp(:,1) + d*nx, wp(:,2) + d*ny];
end
