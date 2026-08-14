% =========================================================================
% 檔案名稱: select_best_path.m
%
% 功能：從 generate_local_paths.m（或 my_multi_path.m）產生的 N 條候選
%       路徑中，依「橫向誤差、曲率、鉸接角安全性」三項指標評分，選出
%       目前最適合追蹤的一條，回傳其索引 best_idx。main_pure_pursuit_sim.m
%       每次 replan（重新生成候選路徑）之後，都會呼叫本函式做選擇。
%
%       本函式完全不知道候選路徑是「整條路線版」還是「即時局部窗口版」，
%       它只認得傳進來的 path_candidates cell array 本身 —— 這正是模組化
%       設計的好處：不論上層架構怎麼改（例如未來換成即時局部生成），
%       選路評分邏輯完全不用動。
%
% 評分公式：
%       score_i = w_cte * cte_i + w_kappa * curv_i + w_hitch * current_hitch
%
%   三項成本個別意義：
%     cte_i        : 候選路徑 i 跟車輛目前位置的橫向誤差（越小代表車輛
%                    離這條路徑越近，越不需要大幅修正就能上線）
%     curv_i       : 候選路徑 i 的「最大曲率」（見下方「為什麼用最大值
%                    不用平均值」的說明）
%     current_hitch: 車輛「目前」的實際鉸接角（跟候選路徑 i 是哪一條
%                    無關，三條候選路徑的這一項都相同）—— 這一項的用意
%                    是「車輛現在折角越大，整體評分基準就越保守」，
%                    但不會改變候選路徑之間的相對排名（因為對所有 i
%                    都加了同一個常數）。真正決定「選哪條」的是前兩項
%                    （cte、curv）與 check_hitch_angle 的可行性篩選。
%
%   為什麼曲率用「最大值」而不是「平均值」：
%     貨櫃的橫向偏移量（off-tracking）主要由候選路徑上「單一最尖銳彎道」
%     的瞬時曲率決定，不是整條路徑的平均曲率。如果用平均值，一條路徑
%     即使只有一小段特別急的彎，平均下來也可能跟緩和彎道的路徑差不多，
%     scorer 就分不出「哪條路徑在最緊的彎道能讓貨櫃少甩尾」，等於失去
%     了讓 scorer 自動選出「彎道外側、曲率較小」那條候選路徑的能力。
%
% 可行性篩選（check_hitch_angle）：
%   只有通過 check_hitch_angle 判定為可行（feas=true）的候選路徑，才會
%   被賦予實際分數；不可行的路徑分數維持初始值 inf，min() 自然不會選到。
%   若「全部」候選路徑都被判定不可行（scores 全部是 inf），退回選擇
%   最中間那條（index = ceil(N/2)，通常對應偏移量 0、最接近母路徑中心線
%   的候選路徑）—— 這是最後一道防線，避免無解時整個選路機制當機。
%
% 輸入：
%   path_candidates : cell array，每個元素是候選路徑 struct（.x .y .phi
%                      .kappa .v_profile），可能包含空陣列 [] 代表該候選
%                      路徑生成失敗（見 stitch_local_path.m 的容錯處理）
%   trailer_state   : [x_tractor, y_tractor, yaw_tractor, yaw_trailer, v]
%                      車輛目前完整狀態（拖車頭位置/航向、貨櫃航向、速度）
%   params          : 需要 w_cte, w_kappa, w_hitch, L2, phi_max
%
% 輸出：
%   best_idx : 分數最低（最適合追蹤）的候選路徑索引
% =========================================================================

function best_idx = select_best_path(path_candidates, trailer_state, params)
    N = numel(path_candidates);
    scores = inf(1, N);

    yaw0 = trailer_state(3);
    yaw1 = trailer_state(4);
    d_hitch = yaw0 - yaw1;
    current_hitch = abs(atan2(sin(d_hitch), cos(d_hitch)));   % 正確 wrap 到 (-pi, pi]

    for i = 1:N
        p = path_candidates{i};
        if isempty(p) || ~isfield(p,'kappa'), continue; end

        cte  = compute_cte(p, trailer_state);
        feas = check_hitch_angle(p, trailer_state, params);
        % 用「最大曲率」而非平均曲率：貨櫃偏移量主要由單一最尖銳彎道的瞬時曲率決定，
        % 用平均值會把整條候選路徑唯一的尖峰稀釋掉，導致 scorer 分不出哪條候選路徑
        % 在最緊的彎道能讓聯結車少甩尾（也就是無法自動選出「彎道外側、曲率較小」的那條）
        curv = max(abs(p.kappa));

        % 聯結角改善項：車輛目前折角越大，整體評分基準越保守（此項對所有候選路徑相同）
        hitch_penalty = current_hitch * params.w_hitch;

        if feas
            scores(i) = params.w_cte * cte + ...
                        params.w_kappa * curv + ...
                        hitch_penalty;
        end
    end
    [~, best_idx] = min(scores);
    if isinf(scores(best_idx))
        best_idx = ceil(N/2);  % 全不可行時選中間路徑（最後防線）
    end
end

% -------------------------------------------------------------------------
% 子函式：compute_cte — 計算候選路徑跟車輛目前位置的橫向誤差
% -------------------------------------------------------------------------
% 原理：先在候選路徑上找出離車輛最近的點 idx（最近點搜尋），取該點的
% 航向角 yaw_ref 當作局部參考方向，再把「車輛位置 - 最近點位置」這個
% 向量投影到路徑法線方向 (-sin(yaw_ref), cos(yaw_ref))，得到的純量就是
% 橫向誤差（跟 pure_pursuit_controller.m 計算 CTE 用的是同一個投影公式）。
function cte = compute_cte(path, trailer_state)
% path: 含 .x .y 的 refpath 結構
% trailer_state: [x_tractor, y_tractor, yaw_tractor, yaw_trailer, v]
    x0 = trailer_state(1); y0 = trailer_state(2);
    yaw0 = trailer_state(3);

    dist2 = (path.x - x0).^2 + (path.y - y0).^2;
    [~, idx] = min(dist2);
    yaw_ref = path.phi(idx);
    dx = x0 - path.x(idx); dy = y0 - path.y(idx);
    cte = abs(-sin(yaw_ref)*dx + cos(yaw_ref)*dy);   % 橫向誤差 = 位置差在路徑法線方向的投影
end

% -------------------------------------------------------------------------
% 子函式：check_hitch_angle — 判斷追蹤此候選路徑是否會讓鉸接角超出安全上限
% -------------------------------------------------------------------------
function ok = check_hitch_angle(path, trailer_state, params)
% 檢查追蹤此路徑時，聯結角是否會超出限制
% 用路徑最大曲率估算折角 (快速近似)
    yaw0 = trailer_state(3);
    yaw1 = trailer_state(4);
    d_hitch = yaw0 - yaw1;
    hitch_angle = abs(atan2(sin(d_hitch), cos(d_hitch)));   % 車輛「目前」的實際鉸接角，正確 wrap 到 (-pi, pi]

    % 路徑最大曲率對應的最大聯結角估算
    max_kappa = max(abs(path.kappa));
    % 幾何近似（穩態圓周運動）：max_hitch ≈ atan(L2 * max_kappa)
    % 注意：不能用 asin(L2*max_kappa) — 當 L2*kappa 接近/超過 1（本車 L2=7.5m、
    % 物理曲率上限 kmax=tan(35°)/L1≈0.156 時 L2*kmax≈1.17>1）asin 會直接飽和到
    % 接近 90°，導致幾乎任何曲率都被誤判為不可行、被迫每次都退回置中路徑
    % —— 這是這次修正的一個關鍵 bug，完整推導見 compute_hitch_speed_cap.m 檔頭。
    % atan() 對任意輸入都平滑，不會有這個飽和問題。
    predicted_max_hitch = atan(params.L2 * max_kappa);

    ok = (hitch_angle < params.phi_max) && ...                  % 目前鉸接角本身沒有超標
         (predicted_max_hitch < params.phi_max * 0.85);         % 追這條路徑「預估」也不會把鉸接角推過安全門檻（85%餘量）
end
