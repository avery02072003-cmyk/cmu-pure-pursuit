% select_best_path.m — 聯結車版本
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
        
        % 聯結角改善項：選能讓折角回正的路徑
        hitch_penalty = current_hitch * params.w_hitch;
        
        if feas
            scores(i) = params.w_cte * cte + ...
                        params.w_kappa * curv + ...
                        hitch_penalty;
        end
    end
    [~, best_idx] = min(scores);
    if isinf(scores(best_idx))
        best_idx = ceil(N/2);  % 全不可行時選中間路徑
    end
end

function cte = compute_cte(path, trailer_state)
% path: 含 .x .y 的 refpath 結構
% trailer_state: [x_tractor, y_tractor, yaw_tractor, yaw_trailer, v]
    x0 = trailer_state(1); y0 = trailer_state(2);
    yaw0 = trailer_state(3);
    
    dist2 = (path.x - x0).^2 + (path.y - y0).^2;
    [~, idx] = min(dist2);
    yaw_ref = path.phi(idx);
    dx = x0 - path.x(idx); dy = y0 - path.y(idx);
    cte = abs(-sin(yaw_ref)*dx + cos(yaw_ref)*dy);
end

function ok = check_hitch_angle(path, trailer_state, params)
% 檢查追蹤此路徑時，聯結角是否會超出限制
% 用路徑最大曲率估算折角 (快速近似)
    yaw0 = trailer_state(3);
    yaw1 = trailer_state(4);
    d_hitch = yaw0 - yaw1;
    hitch_angle = abs(atan2(sin(d_hitch), cos(d_hitch)));   % 正確 wrap 到 (-pi, pi]
    
    % 路徑最大曲率對應的最大聯結角估算
    max_kappa = max(abs(path.kappa));
    % 幾何近似（穩態圓周運動）：max_hitch ≈ atan(L2 * max_kappa)
    % 注意：不能用 asin(L2*max_kappa) — 當 L2*kappa 接近/超過 1（本車 L2=7.5m、
    % 物理曲率上限 kmax=tan(35°)/L1≈0.156 時 L2*kmax≈1.17>1）asin 會直接飽和到
    % 接近 90°，導致幾乎任何曲率都被誤判為不可行、被迫每次都退回置中路徑。
    % atan() 對任意輸入都平滑，不會有這個問題。
    predicted_max_hitch = atan(params.L2 * max_kappa);
    
    ok = (hitch_angle < params.phi_max) && ...
         (predicted_max_hitch < params.phi_max * 0.85);  % 85% 安全餘量
end