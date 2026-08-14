% generate_local_paths.m — 即時局部多候選路徑生成
% 每次呼叫都先對母路徑做一次最近點搜尋，找到車輛「目前位置」附近的錨點，
% 往母路徑前方取一段窗口，重新產生 N_paths 條側向偏移候選路徑，
% 取代「整條路線只算一次候選集合」的舊架構。
%
% 純函式，不依賴全域變數／檔案 I/O，之後 Frenet/MPC 可直接複用。
%
% 輸入：
%   state     : [x, y, yaw] 車輛目前位姿（錨點）
%   refpath   : 母路徑 struct，含 .x .y（環形路線，首尾相接）
%   params    : 需要 lane_width, local_horizon_m, local_wp_spacing, local_sample_stride,
%               a_lat_max/a_acc_max/a_dec_max/v_des/v_profile_min, L1, L2, phi_max,
%               hitch_speed_cap_frac/gain
%   N_paths   : 候選路徑數量（可調，需求是 >=3）
%   obstacles : 保留參數，目前傳 [] / {}；之後路障功能直接接這個介面，不必再改函式簽名
%
% 輸出：
%   path_candidates : cell array，每個元素是 refpath 結構 (.x .y .phi .kappa .v_profile)，
%                      格式與 pure_pursuit_controller.m / select_best_path.m 現有介面相同

function path_candidates = generate_local_paths(state, refpath, params, N_paths, obstacles) %#ok<INUSD>
    Nref = length(refpath.x);

    % ---- 1. 全域最近點搜尋，找車輛在母路徑上的錨定索引 ----
    dist_all = hypot(refpath.x - state(1), refpath.y - state(2));
    [~, idx_near] = min(dist_all);

    % ---- 2. 從錨定索引往前走，累積弧長直到涵蓋 local_horizon_m（環形路線用 mod 處理接縫）----
    idx_list = idx_near;
    arc = 0;
    i_cur = idx_near;
    while arc < params.local_horizon_m && numel(idx_list) < Nref
        i_nxt = mod(i_cur, Nref) + 1;
        arc = arc + hypot(refpath.x(i_nxt)-refpath.x(i_cur), refpath.y(i_nxt)-refpath.y(i_cur));
        idx_list(end+1) = i_nxt; %#ok<AGROW>
        i_cur = i_nxt;
    end

    window_x   = refpath.x(idx_list);
    window_y   = refpath.y(idx_list);
    window_phi = unwrap(refpath.phi(idx_list));   % 沿窗口 unwrap，避免跨 -pi/+pi 內插出錯

    % 濾掉近乎重複的點，避免下面 interp1 對重複弧長座標出錯
    d_step = hypot(diff(window_x), diff(window_y));
    keep = [true; d_step > 1e-9];
    window_x   = window_x(keep);
    window_y   = window_y(keep);
    window_phi = window_phi(keep);

    % ---- 3. 依 local_wp_spacing 抽稀成局部 waypoints ----
    % 航向直接內插母路徑既有、平滑的 refpath.phi（而不是用稀疏點的 gradient() 重新估計），
    % 避免粗抽稀造成航向估計雜訊、進而讓 my_path() 擬合出過度彎曲的路徑
    s_win = [0; cumsum(hypot(diff(window_x), diff(window_y)))];
    s_targets = (0:params.local_wp_spacing:s_win(end))';
    if isempty(s_targets) || s_targets(end) < s_win(end)
        s_targets(end+1,1) = s_win(end);
    end
    local_x   = interp1(s_win, window_x,   s_targets, 'linear');
    local_y   = interp1(s_win, window_y,   s_targets, 'linear');
    local_phi = interp1(s_win, window_phi, s_targets, 'linear');
    local_wp = [local_x, local_y];
    phi_wp   = local_phi;

    % 注意：窗口起點刻意「不」強制改成車輛的精確瞬時位置/航向。
    % 起點就是母路徑上離車輛最近的點（第 1 步的全域最近點搜尋已保證很接近車輛實際位置，
    % 差距通常就是當下的 CTE，一般是次公尺級），讓每段路徑都能保持平滑銜接；
    % 若強制讓第一段在很短的弧長內同時修正位置與航向誤差，會逼出接近物理極限的曲率尖峰，
    % 這件事本身應該交給追蹤控制器（look-ahead 內的漸進修正）處理，不該由路徑幾何硬扛。
    % 「即時、以車輛當前位置為錨點重新生成」的需求，靠每次 replan 都重新做最近點搜尋來達成。

    % ---- 4. 產生 N_paths 條側向偏移候選路徑 ----
    offsets = linspace(-params.lane_width/2, params.lane_width/2, N_paths);
    path_candidates = cell(1, N_paths);
    win = 9;   % 與 compute_path_curvature 的平滑視窗一致，供防呆用

    for i = 1:N_paths
        wp_shifted = shift_waypoints_lateral(local_wp, offsets(i));

        [path_x, path_y, path_phi] = stitch_local_path(wp_shifted, phi_wp, params.local_sample_stride);

        if isempty(path_x) || length(path_x) < win
            path_candidates{i} = [];
            continue;
        end

        cand.x = path_x;
        cand.y = path_y;
        cand.phi = path_phi;
        cand.kappa = compute_path_curvature(path_x, path_y, path_phi, params);
        cand.v_profile = compute_v_profile(cand.kappa, params);
        path_candidates{i} = cand;
    end
end
