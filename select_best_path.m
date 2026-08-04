% select_best_path.m — 聯結車版本
function best_idx = select_best_path(path_candidates, trailer_state, params)
    N = numel(path_candidates);
    scores = inf(1, N);
    
    yaw0 = trailer_state(3);
    yaw1 = trailer_state(4);
    current_hitch = abs(yaw0 - yaw1);
    
    for i = 1:N
        p = path_candidates{i};
        if isempty(p) || ~isfield(p,'kappa'), continue; end
        
        cte  = compute_cte(p, trailer_state);
        feas = check_hitch_angle(p, trailer_state, params);
        curv = mean(abs(p.kappa));   % 用平均曲率（比 max 穩定）
        
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
    hitch_angle = abs(yaw0 - yaw1);
    
    % 路徑最大曲率對應的最大聯結角估算
    max_kappa = max(abs(path.kappa));
    % 幾何近似：max_hitch ≈ asin(L2 * max_kappa)
    predicted_max_hitch = asin(min(params.L2 * max_kappa, 0.999));
    
    ok = (hitch_angle < params.phi_max) && ...
         (predicted_max_hitch < params.phi_max * 0.85);  % 85% 安全餘量
end