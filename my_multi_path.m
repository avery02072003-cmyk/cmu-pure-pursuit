function path_candidates = my_multi_path(gps_waypoints, N_paths, params)
    % gps_waypoints: Nx2 矩陣 [x, y]
    % 回傳 cell array，每個元素是 refpath 結構 (x, y, phi, kappa, v_profile)
    %
    % 整條路線一次性生成（供 STEP2_MultiPathGen.m 預覽用）。核心的側向平移／
    % 逐段路徑拼接／曲率／速度規劃邏輯已抽出成獨立函式，跟即時局部生成
    % generate_local_paths.m 共用同一套（已修正過轉置錯誤的）核心邏輯，
    % 避免兩處各自維護一份容易分岔的重複程式碼。
    win = 9;   % 供防呆用，需與 compute_path_curvature 的平滑視窗一致
    sample_stride = 25;   % 每段 1000 個取樣點降到 ~40 點，整條路線預覽仍平滑但不會產生過量資料

    path_candidates = cell(1, N_paths);
    offsets = linspace(-params.lane_width/2, params.lane_width/2, N_paths);

    % 從 GPS waypoints 估算初始航向角
    dx_wp = gradient(gps_waypoints(:,1));
    dy_wp = gradient(gps_waypoints(:,2));
    phi_wp = atan2(dy_wp, dx_wp);

    for i = 1:N_paths
        wp_shifted = shift_waypoints_lateral(gps_waypoints, offsets(i));

        [path_x, path_y, path_phi] = stitch_local_path(wp_shifted, phi_wp, sample_stride);

        if isempty(path_x) || length(path_x) < win
            path_candidates{i} = [];   % 標記此候選路徑無效
            continue;   % 跳過這條路徑，直接進入下一個 i
        end

        % 存成 refpath 結構
        cand.x = path_x;
        cand.y = path_y;
        cand.phi = path_phi;
        cand.kappa = compute_path_curvature(path_x, path_y, path_phi, params);
        cand.v_profile = compute_v_profile(cand.kappa, params);
        path_candidates{i} = cand;
    end
end
