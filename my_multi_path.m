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
        xic = [0, 0, 0, 0];   % 統一從局部原點、局部朝向 0 開始（第一段跟後續段落邏輯一致）
        
        for j = 1:size(wp_shifted,1)-1
        % 世界座標系的原始位移
        dx_world = wp_shifted(j+1,1) - wp_shifted(j,1);
        dy_world = wp_shifted(j+1,2) - wp_shifted(j,2);

        % 目前這段的起點航向角（局部座標的基準）
        phi_start = phi_wp(j);

        % ---- 旋轉：把世界座標位移轉到「起點朝向=0」的局部座標 ----
        c = cos(-phi_start); s = sin(-phi_start);
        x_tag = c*dx_world - s*dy_world;
        y_tag = s*dx_world + c*dy_world;

        % 目標航向角改為「相對變化量」，不是絕對角度
        phi_tag = phi_wp(j+1) - phi_start;
        phi_tag = atan2(sin(phi_tag), cos(phi_tag));  % 包進 -pi~pi

        try
            [seg, ~] = my_path(x_tag, y_tag, phi_tag, xic);

            % ---- 把局部座標的曲線點轉回世界座標 ----
            c2 = cos(phi_start); s2 = sin(phi_start);
            seg_x_world = c2*seg(:,1) - s2*seg(:,2) + wp_shifted(j,1);
            seg_y_world = s2*seg(:,1) + c2*seg(:,2) + wp_shifted(j,2);
            seg_phi_world = seg(:,3) + phi_start;

            path_x   = [path_x;   seg_x_world];
            path_y   = [path_y;   seg_y_world];
            path_phi = [path_phi; seg_phi_world];

            xic = [0, 0, 0, 0];   % 下一段永遠從局部原點、局部朝向0開始
        catch
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
    v_c = max(v_c, params.v_profile_min);
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