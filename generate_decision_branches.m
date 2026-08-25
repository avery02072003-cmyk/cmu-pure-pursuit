% =========================================================================
% 檔案名稱: generate_decision_branches.m
%
% 功能：離散車道決策分支路徑生成器的核心。給定車輛目前狀態與所在車道，
%       產生 2 或 3 條候選「分支路徑」，每條分支代表一種可能的下一步
%       決策，末端就是下一個決策點（畫成 X 標記，見
%       init_decision_graph_plot.m / update_decision_graph_plot.m）。
%
%       跟既有的 generate_local_paths.m（N 條連續平行偏移候選路徑）是
%       兩種不同的路徑生成哲學，彼此獨立、互不影響：
%         - generate_local_paths.m：車道內／跨鄰車道的「連續」偏移量，
%           用於 main_pure_pursuit_sim.m 的即時追蹤與（未來的）障礙物
%           評分選路。
%         - generate_decision_branches.m（本檔案）：固定只有左/中/右
%           3 條「離散」車道身分，用於視覺化「現在有哪些選擇、選了會
%           走到哪裡」，目前只有 STEP3_DecisionGraphDemo.m 使用。
%
% 分支規則（固定，不依賴 n_side_lanes 泛化）：
%   current_lane_id =  0（中間車道）→ 3 條分支：change_right(-1)、
%                       straight(0)、change_left(+1)
%   current_lane_id = +1（左車道）  → 2 條分支：change_right(0，變回中間)、
%                       straight(+1)
%   current_lane_id = -1（右車道）  → 2 條分支：straight(-1)、
%                       change_left(0，變回中間)
%   （車道代號正值＝左、負值＝右，跟 shift_waypoints_lateral.m 的偏移
%   正負號慣例一致；type 命名反映「動作方向」：往左手邊移動＝change_left，
%   不管是從中間車道變到左車道、還是從右車道變回中間車道，都是
%   change_left。）
%
% 決策點距離（跟速度成比例，上限 30m，見 STEP1_VehicleParameters.m 第10節）：
%       decision_spacing_m = min(v * decision_lookahead_time_s, decision_spacing_max_m)
%
% 直行分支：直接複用既有的 shift_waypoints_lateral.m（常數偏移）+
%   stitch_local_path.m，做法跟 generate_local_paths.m 的單一候選路徑
%   完全相同。
%
% 換道分支：側向偏移量不是常數，而是用 lane_change_offset_profile.m
%   算出的 S 型斜坡（從目前車道偏移平滑過渡到目標車道偏移），斜坡本身
%   的變化率會產生真實的航向偏轉（dphi_correction，完整推導見
%   lane_change_offset_profile.m 檔頭），必須加進目標航向角，
%   stitch_local_path.m 才能擬合出真正平滑、不會在 segment 交界處
%   出現稜角的換道曲線。換道斜坡區段用比一般直行區段更密的取樣間距
%   （lane_change_wp_spacing=0.5m vs local_wp_spacing=1.5m），原因同樣
%   是航向變化率較高的區段需要較密取樣才不會在視覺上出現稜角。
%
% 輸入：
%   state           : [x, y, yaw, v] 車輛目前狀態
%   refpath         : 母路徑 struct，含 .x .y .phi（環形路線，首尾相接）
%   params          : 需要 lane_width, decision_lookahead_time_s,
%                     decision_spacing_max_m, lane_change_time_s,
%                     lane_change_wp_spacing, lane_change_min_distance_m,
%                     local_wp_spacing, local_sample_stride, L1
%                     （L1 供 compute_path_curvature.m 換算物理曲率上限用）
%   current_lane_id : 車輛目前所在車道，-1/0/+1（見 estimate_current_lane_id.m）
%
% 輸出：
%   branches : struct array（2 或 3 個元素），每個元素含：
%     .type           'straight' | 'change_left' | 'change_right'
%     .to_lane_id     這條分支走到底會抵達的車道代號
%     .x .y .phi      世界座標系下的分支路徑點（從 state 位置到決策點）
%     .decision_point [x_end, y_end]，這條分支末端的 X 標記座標
%     .distance_m     這條分支到決策點的實際延伸距離（=本次呼叫用到的
%                      decision_spacing_m，供畫圖標示用）
% =========================================================================

function branches = generate_decision_branches(state, refpath, params, current_lane_id)
    v = state(4);
    decision_spacing_m = min(v * params.decision_lookahead_time_s, params.decision_spacing_max_m);
    decision_spacing_m = max(decision_spacing_m, params.lane_change_min_distance_m);  % 數值安全下限

    % ---- 依目前車道決定要生成哪些分支（目標車道 + 對應動作類型）----
    if current_lane_id == 0
        to_lane_ids = [-1, 0, 1];
    elseif current_lane_id == 1
        to_lane_ids = [0, 1];
    else % current_lane_id == -1
        to_lane_ids = [-1, 0];
    end

    % ---- 窗口只需要抽一次，所有分支共用同一段母路徑窗口 ----
    [wx, wy, wphi, s_win] = sample_refpath_window(state(1:2), refpath, decision_spacing_m);

    branches = struct('type', {}, 'to_lane_id', {}, 'x', {}, 'y', {}, 'phi', {}, 'decision_point', {}, 'distance_m', {});

    for i = 1:numel(to_lane_ids)
        to_lane_id = to_lane_ids(i);
        offset_start = current_lane_id * params.lane_width;
        offset_end   = to_lane_id * params.lane_width;

        if to_lane_id == current_lane_id
            % ---- 直行分支：常數偏移，複用既有模組 ----
            type_str = 'straight';
            s_targets = (0:params.local_wp_spacing:decision_spacing_m)';
            if isempty(s_targets) || s_targets(end) < decision_spacing_m
                s_targets(end+1,1) = decision_spacing_m;
            end
            [local_x, local_y, local_phi] = resample_window_by_arclength(wx, wy, wphi, s_win, s_targets);
            local_wp = [local_x, local_y];

            wp_shifted = shift_waypoints_lateral(local_wp, offset_start);
            [path_x, path_y, path_phi] = stitch_local_path(wp_shifted, local_phi, params.local_sample_stride);
        else
            % ---- 換道分支：S 型偏移斜坡 + 航向修正 ----
            if to_lane_id > current_lane_id
                type_str = 'change_left';
            else
                type_str = 'change_right';
            end

            ramp_distance = v * params.lane_change_time_s;
            ramp_distance = max(ramp_distance, params.lane_change_min_distance_m);
            ramp_distance = min(ramp_distance, decision_spacing_m * 0.9);  % 留一段直行尾段，確保換道在到達決策點前完成

            s_ramp   = (0:params.lane_change_wp_spacing:ramp_distance)';
            s_tail   = ((ramp_distance+params.local_wp_spacing):params.local_wp_spacing:decision_spacing_m)';
            s_targets = unique([s_ramp; s_tail; ramp_distance; decision_spacing_m]);

            [local_x, local_y, local_phi] = resample_window_by_arclength(wx, wy, wphi, s_win, s_targets);
            local_wp = [local_x, local_y];

            [d_profile, dphi_corr] = lane_change_offset_profile(s_targets, ramp_distance, offset_start, offset_end);
            wp_shifted = shift_waypoints_lateral_profile(local_wp, d_profile);
            phi_branch = local_phi + dphi_corr;
            phi_branch = atan2(sin(phi_branch), cos(phi_branch));   % wrap 到 (-pi, pi]

            [path_x, path_y, path_phi] = stitch_local_path(wp_shifted, phi_branch, params.local_sample_stride);
        end

        if isempty(path_x)
            % 保底：極端情況下 stitch_local_path 內部 my_path() 沒有一段收斂
            % （5倍放大後的平緩賽道理論上不該發生，這裡只是防呆），
            % 退而求其次直接用未經平滑擬合的偏移 waypoints 當作分支路徑，
            % 至少能畫出分支、不會讓整個決策圖生成中斷。
            path_x = wp_shifted(:,1);
            path_y = wp_shifted(:,2);
            path_phi = local_phi;
        end

        branches(end+1) = struct(...
            'type', type_str, ...
            'to_lane_id', to_lane_id, ...
            'x', path_x, 'y', path_y, 'phi', path_phi, ...
            'decision_point', [path_x(end), path_y(end)], ...
            'distance_m', decision_spacing_m); %#ok<AGROW>
            % distance_m：這條分支到「下一個決策點」實際延伸的距離。目前同一次
            % generate_decision_branches() 呼叫裡，所有分支的 distance_m 都相同
            % （都是同一個 decision_spacing_m），分開存成每條分支自己的欄位是為了
            % 讓 update_decision_graph_plot.m 可以直接讀 branches(i).distance_m
            % 標在對應的 X 標記旁邊，不需要额外傳遞 decision_spacing_m，也讓未來
            % 如果改成「每條分支距離可以不同」時不用再改資料結構。
    end
end
