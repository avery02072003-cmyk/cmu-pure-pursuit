function [delta, idx_target, idx_near, Ld, alpha] = pure_pursuit_controller(x, y, yaw, v, refpath, params, idx_prev)

N = length(refpath.x);
dist_all = hypot(refpath.x - x, refpath.y - y);

search_len = 80;
search_range = mod((idx_prev-1):(idx_prev-1+search_len), N) + 1;
[~, local_min] = min(dist_all(search_range));
idx_near = search_range(local_min);

kappa_near = 0;
if isfield(refpath, 'kappa')
    kappa_near = abs(refpath.kappa(idx_near));
end

Ld = params.Ld0 + params.kv * abs(v) - params.kappa_gain * kappa_near;
Ld = max(params.Ld_min, min(Ld, params.Ld_max));

% --- cubic preview: 從 idx_near 往前取 preview 段，
%     在 Ld 距離處用 cubic spline 計算局部目標點位置與切線方向 ---
preview_len = min(60, N - 1);
idx_range = mod((idx_near-1):(idx_near-1+preview_len), N) + 1;

px = refpath.x(idx_range);
py = refpath.y(idx_range);

% 弧長參數化
ds_seg = hypot(diff(px), diff(py));
ds_seg(ds_seg < 1e-6) = 1e-6;
s_seg = [0; cumsum(ds_seg(:))];

if s_seg(end) < Ld
    idx_target = idx_range(end);
    tx = refpath.x(idx_target);
    ty = refpath.y(idx_target);
    yaw_target = refpath.phi(idx_target);
else
    % cubic spline 插值在 s = Ld 處取目標點
    tx = interp1(s_seg, px, Ld, 'pchip');
    ty = interp1(s_seg, py, Ld, 'pchip');

    % 切線方向：對 spline 求導數估計局部 yaw
    ds_query = 0.05;
    tx2 = interp1(s_seg, px, min(Ld + ds_query, s_seg(end)), 'pchip');
    ty2 = interp1(s_seg, py, min(Ld + ds_query, s_seg(end)), 'pchip');
    yaw_target = atan2(ty2 - ty, tx2 - tx);

    % idx_target 找最近的參考點索引（用於記錄）
    dist_target = hypot(refpath.x - tx, refpath.y - ty);
    [~, idx_target] = min(dist_target);
end

% --- pure pursuit 幾何 ---
alpha = atan2(ty - y, tx - x) - yaw;
alpha = atan2(sin(alpha), cos(alpha));
delta_pp = atan2(2 * params.L * sin(alpha), Ld);

% --- heading feedback（對齊 cubic preview 的切線方向，而不是 idx_near）---
he_now = atan2(sin(yaw - yaw_target), cos(yaw - yaw_target));
v_safe = max(v, 1.0);
delta_fb = -params.Kh * he_now / v_safe;

delta = delta_pp + delta_fb;

delta_max_phys = deg2rad(35);
delta = max(-delta_max_phys, min(delta, delta_max_phys));

% --- 側向加速度硬限制 ---
kappa_cmd = tan(delta) / params.L;
a_lat_cmd  = v^2 * kappa_cmd;
if abs(a_lat_cmd) > params.a_lat_max
    kappa_limited = sign(a_lat_cmd) * params.a_lat_max / max(v^2, 1e-3);
    delta = atan(kappa_limited * params.L);
end

delta = max(-delta_max_phys, min(delta, delta_max_phys));

end