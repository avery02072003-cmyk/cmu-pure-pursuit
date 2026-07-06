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

idx_target = idx_near;
for step_i = 0:N-1
    i = mod(idx_near - 1 + step_i, N) + 1;
    if dist_all(i) >= Ld
        idx_target = i;
        break;
    end
end

tx = refpath.x(idx_target);
ty = refpath.y(idx_target);

alpha = atan2(ty - y, tx - x) - yaw;
alpha = atan2(sin(alpha), cos(alpha));

delta_pp = atan2(2 * params.L * sin(alpha), Ld);

yaw_ref = refpath.phi(idx_near);
he_now = atan2(sin(yaw - yaw_ref), cos(yaw - yaw_ref));

delta = delta_pp - params.Kh * he_now;

delta_max = deg2rad(35);
delta = max(-delta_max, min(delta, delta_max));

end