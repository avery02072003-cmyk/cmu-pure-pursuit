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

% 弧長累加搜尋 idx_target
arc = 0;
idx_target = idx_near;
for step_i = 1:N-1
    i_cur = mod(idx_near - 1 + step_i - 1, N) + 1;
    i_nxt = mod(idx_near - 1 + step_i,     N) + 1;
    arc = arc + hypot(refpath.x(i_nxt) - refpath.x(i_cur), ...
                      refpath.y(i_nxt) - refpath.y(i_cur));
    if arc >= Ld
        idx_target = i_nxt;
        break;
    end
end

tx = refpath.x(idx_target);
ty = refpath.y(idx_target);

% pure pursuit 幾何
alpha = atan2(ty - y, tx - x) - yaw;
alpha = atan2(sin(alpha), cos(alpha));
delta_pp = atan2(2 * params.L * sin(alpha), Ld);

% 座標差分估計 idx_target 切線方向
hw = 2;
i_a = mod(idx_target - 1 - hw, N) + 1;
i_b = mod(idx_target - 1 + hw, N) + 1;
yaw_target = atan2(refpath.y(i_b) - refpath.y(i_a), ...
                   refpath.x(i_b) - refpath.x(i_a));

he_now = atan2(sin(yaw - yaw_target), cos(yaw - yaw_target));
v_safe = max(v, 1.0);
delta_fb = -params.Kh * he_now / v_safe;

delta = delta_pp + delta_fb;

delta_max_phys = deg2rad(35);
delta = max(-delta_max_phys, min(delta, delta_max_phys));

% 側向加速度硬限制
kappa_cmd = tan(delta) / params.L;
a_lat_cmd  = v^2 * kappa_cmd;
if abs(a_lat_cmd) > params.a_lat_max
    kappa_limited = sign(a_lat_cmd) * params.a_lat_max / max(v^2, 1e-3);
    delta = atan(kappa_limited * params.L);
end

delta = max(-delta_max_phys, min(delta, delta_max_phys));

end