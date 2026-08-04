function path_candidates = my_multi_path(gps_waypoints, N_paths, params)
% gps_waypoints: Nx2 矩陣 [x, y]
% 回傳 cell array，每個元素是 refpath 結構 (x, y, phi, kappa, v_profile)
win = 9;   % 供防呆與 compute_kappa 共用的平滑視窗大小

path_candidates = cell(1, N_paths);
offsets = linspace(-params.lane_width/2, params.lane_width/2, N_paths);

% 從 GPS waypoints 估算初始航向角
dx_wp = gradient(gps_waypoints(:,1));
dy_wp = gradient(gps_waypoints(:,2));
phi_wp = atan2(dy_wp, dx_wp);

for i = 1:N_paths
    wp_shifted = shift_waypoints_lateral(gps_waypoints, offsets(i));
    
    % ---- 逐段呼叫 my_path() 連接相鄰 waypoint ----
    path_x = []; path_y = []; path_phi = [];
    xic = [wp_shifted(1,1), wp_shifted(1,2), phi_wp(1), 0];  % 初始狀態
    
    for j = 1:size(wp_shifted,1)-1
        x_tag = wp_shifted(j+1,1) - wp_shifted(j,1);  % 相對座標
        y_tag = wp_shifted(j+1,2) - wp_shifted(j,2);
        phi_tag = phi_wp(j+1);
        
        try
            [seg, ~] = my_path(x_tag, y_tag, phi_tag, xic);
            path_x   = [path_x;   seg(:,1) + wp_shifted(j,1)];
            path_y   = [path_y;   seg(:,2) + wp_shifted(j,2)];
            path_phi = [path_phi; seg(:,3)];
            xic = [0, 0, phi_tag, 0];  % 更新初始條件為下一段起點
        catch
            % Newton-Raphson 不收斂時跳過此段
            continue;
        end
    end

    if isempty(path_x) || length(path_x) < win
        path_candidates{i} = [];   % 標記此候選路徑無效
        continue;   % 跳過這條路徑，直接進入下一個 i
    end
    
    % 存成 refpath 結構
    cand.x = path_x;
    cand.y = path_y;
    cand.phi = path_phi;
    cand.kappa = compute_kappa(path_x, path_y, path_phi, params);
    cand.v_profile = compute_v_profile(cand.kappa, params);
    path_candidates{i} = cand;
end
end

function kappa = compute_kappa(x, y, phi, params)
    ds = hypot(gradient(x), gradient(y));
    ds(ds<1e-6) = 1e-6;
    kappa = gradient(unwrap(phi)) ./ ds;
    kmax = tan(deg2rad(35)) / params.L1;
    kappa = max(min(kappa, kmax), -kmax);
    win = 9;
    kernel = ones(1,win)/win;
    kappa = conv([kappa(1)*ones(1,(win-1)/2), kappa(:)', kappa(end)*ones(1,(win-1)/2)], kernel, 'valid');
    kappa = kappa(:);
end

function v_profile = compute_v_profile(kappa, params)
    N = length(kappa);
    v_c = sqrt(params.a_lat_max ./ max(abs(kappa), 1e-4));
    v_c = min(v_c, params.v_des);
    v_c = max(v_c, params.v_min);
    v_profile = v_c;
    % backward pass
    for i = N-1:-1:1
        v_allow = sqrt(v_profile(i+1)^2 + 2*params.a_dec_max*0.1);
        v_profile(i) = min(v_profile(i), v_allow);
    end
    % forward pass
    for i = 2:N
        v_allow = sqrt(v_profile(i-1)^2 + 2*params.a_acc_max*0.1);
        v_profile(i) = min(v_profile(i), v_allow);
    end
end

function wp_out = shift_waypoints_lateral(wp, d)
    dx = gradient(wp(:,1));
    dy = gradient(wp(:,2));
    heading = atan2(dy, dx);
    nx = -sin(heading);
    ny = cos(heading);
    wp_out = [wp(:,1) + d*nx, wp(:,2) + d*ny];
end