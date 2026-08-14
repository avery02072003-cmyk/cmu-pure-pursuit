% hitch_angle_governor.m — 即時鉸接角安全網
% 監控目前實際鉸接角（正確 wrap 過），一旦逼近 phi_max 就漸進降速（只降速、不動轉向，
% 避免跟 pure pursuit 幾何互相干擾）。作為 compute_hitch_speed_cap 的即時安全網，
% 修正曲率預估殘留誤差／動態追蹤瞬態造成的鉸接角超標。

function [v_ref_out, gov_active, hitch_now] = hitch_angle_governor(v_ref_in, yaw0, yaw1, params)
    d = yaw0 - yaw1;
    hitch_now = abs(atan2(sin(d), cos(d)));   % 正確 wrap 到 (-pi, pi]

    warn_th = params.phi_max * params.hitch_gov_warn_frac;
    hard_th = params.phi_max * params.hitch_gov_hard_frac;

    if hitch_now <= warn_th
        v_ref_out = v_ref_in;
        gov_active = false;
        return;
    end

    frac = min(1, (hitch_now - warn_th) / max(hard_th - warn_th, 1e-6));
    v_ref_out = max(params.v_profile_min, v_ref_in * (1 - frac));
    gov_active = true;
end
