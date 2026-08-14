% compute_hitch_speed_cap.m — 依曲率預估聯結角，換算成鉸接角限速 v_hitch
% 與 select_best_path.m 的 check_hitch_angle 用同一個幾何近似（穩態圓周運動）：
% predicted_hitch ≈ atan(L2*kappa)（不能用 asin，理由見 select_best_path.m 的註解：
% L2*kappa 在本車幾何下可超過 1，asin 會飽和到接近 90°）。
% 類比既有 a_lat_max -> v_curve 的做法，這裡是 phi_max -> v_hitch，
% 讓聯結車在還沒轉進急彎前就先降速，避免鉸接角在動態追蹤中逼近 phi_max。

function v_hitch = compute_hitch_speed_cap(kappa, params)
    predicted_hitch = atan(params.L2 * abs(kappa));
    margin = params.phi_max * params.hitch_speed_cap_frac;   % margin 以下不限速
    over = max(0, predicted_hitch - margin) ./ max(params.phi_max - margin, 1e-6);
    v_hitch = params.v_des ./ (1 + params.hitch_speed_cap_gain * over);  % 平滑降速
end
