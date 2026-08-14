% compute_v_profile.m — 由曲率剖面計算候選路徑的速度規劃
% 抽出自 my_multi_path.m 的 compute_v_profile，並新增 compute_hitch_speed_cap 鉸接角限速項

function v_profile = compute_v_profile(kappa, params)
    N = length(kappa);
    v_lat   = sqrt(params.a_lat_max ./ max(abs(kappa), 1e-4));   % 側向加速度限速
    v_hitch = compute_hitch_speed_cap(kappa, params);            % 鉸接角限速
    v_c = min(min(v_lat, v_hitch), params.v_des);
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
