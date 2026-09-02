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
% 回收模式（recovery mode）—— 這次新增，解決「候選都離車輛很遠時沒有
% 導回中心的邏輯」這個缺口：
%   generate_local_paths.m 每次 replan 都用「車輛目前位置」重新做母路徑
%   最近點搜尋當錨點，候選路徑就是這個錨點左右各偏移一點的平行曲線。
%   正常情況下（cte 小）這完全沒問題：cte minimization 自然會選中
%   offset≈0 的候選，因為那條剛好也離車輛最近。但一旦車輛已經偏出候選
%   涵蓋範圍（例如過彎太快、center 候選一度被 check_hitch_angle 判定
%   不可行，被迫選了曲率較緩的極端候選），問題就出現了：candidate i 的
%   cte_i 本質上量的是「車輛目前位置」跟「這個新錨點 + offset_i」的距離，
%   而這個錨點本身已經是從車輛（已經偏移的）位置重新搜尋出來的——所以
%   cte 最小的候選，幾乎必然又是「offset 剛好匹配車輛目前已經偏移量」
%   的那條，而不是「真正的車道中心線」。如果那個造成 center 不可行的
%   原因（例如車速還沒完全降下來）在下一次 replan 還沒解除，同一套邏輯
%   會再選一次同一側的極端候選——於是車輛在每個新錨點上不斷重複選到
%   同一側，跟真正的母路徑中心線越差越遠，形成棱形發散（實測：CTE 從
%   數公尺内一路發散到 50+ 公尺，過程中轉向角卻只有幾度——代表車輛把
%   選中的候選路徑追得很準，問題出在候選本身已經偏離真正車道中心線）。
%
%   偵測方式：檢查所有「可行」候選裡最小的 cte 是否已經超過一個車道寬
%   （params.lane_width）。正常追蹤時 cte 遠小於這個門檻，這個判斷完全
%   不會觸發，不影響既有行為；只有真的已經偏出候選涵蓋範圍太多時才會
%   觸發。觸發後，評分不再看 cte（此時 cte 已經不具鑑別力，見上面的
%   推導——它只會不斷確認車輛目前已經偏移到哪，而不是引導車輛修正）
%   ，改成直接偏好 |offset_i| 最小（也就是最接近真正母路徑中心線）的
%   可行候選，讓車輛能一步步往回收斂，而不是每次都被鎖定在同一側。
%
% 輸入：
%   path_candidates : cell array，每個元素是候選路徑 struct（.x .y .phi
%                      .offset .kappa .v_profile），可能包含空陣列 []
%                      代表該候選路徑生成失敗（見 stitch_local_path.m 的
%                      容錯處理）。.offset 供回收模式判斷用，由
%                      generate_local_paths.m / my_multi_path.m 產生
%   trailer_state   : [x_tractor, y_tractor, yaw_tractor, yaw_trailer, v]
%                      車輛目前完整狀態（拖車頭位置/航向、貨櫃航向、速度）
%   params          : 需要 w_cte, w_kappa, w_hitch, L2, phi_max, lane_width
%                      （lane_width 是回收模式的觸發門檻，見上方說明）
%
% 輸出：
%   best_idx : 分數最低（最適合追蹤）的候選路徑索引；回收模式時則是
%              可行候選裡 |offset| 最小（最接近母路徑中心線）的索引
% =========================================================================

function best_idx = select_best_path(path_candidates, trailer_state, params)
    N = numel(path_candidates);
    scores      = inf(1, N);
    cte_vals    = inf(1, N);   % 只有可行候選才填入實際 cte，供回收模式偵測門檻用
    offset_vals = zeros(1, N);

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
            cte_vals(i)    = cte;
            offset_vals(i) = p.offset;
            scores(i) = params.w_cte * cte + ...
                        params.w_kappa * curv + ...
                        hitch_penalty;
        end
    end

    if all(isinf(scores))
        best_idx = ceil(N/2);  % 全不可行時選中間路徑（最後防線）
        return;
    end

    if min(cte_vals) > params.lane_width
        % ---- 回收模式：連最好的可行候選都離車輛超過一個車道寬，cte 已經
        % 不具鑑別力（見檔頭推導），改成直接偏好離母路徑中心線最近
        % （|offset| 最小）的可行候選，引導車輛逐次 replan 收斂回中心，
        % 而不是重複鎖定在偏移後的同一側 ----
        offset_pick = abs(offset_vals);
        offset_pick(isinf(scores)) = inf;   % 不可行的候選排除在外
        [~, best_idx] = min(offset_pick);
    else
        [~, best_idx] = min(scores);
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
