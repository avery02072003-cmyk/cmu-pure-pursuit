function [delta, idx_target, idx_near, Ld] = pure_pursuit_controller(x, y, yaw, v, refpath, params, idx_prev)

N = length(refpath.x);
dist_all = sqrt((refpath.x - x).^2 + (refpath.y - y).^2);

search_range = mod((idx_prev-1):(idx_prev-1+200), N) + 1;
[~, local_min] = min(dist_all(search_range));
idx_near = search_range(local_min);

Ld = params.Ld0 + params.kv * abs(v);
Ld = max(params.Ld_min, min(Ld, params.Ld_max));

idx_target = idx_near;
found = false;
for step_i = 0:N-1
    i = mod(idx_near - 1 + step_i, N) + 1;
    d = sqrt((refpath.x(i)-x)^2 + (refpath.y(i)-y)^2);
    if d >= Ld
        idx_target = i;
        found = true;
        break;
    end
end
if ~found
    idx_target = mod(idx_near, N) + 1;
end

tx = refpath.x(idx_target);
ty = refpath.y(idx_target);

alpha = atan2(ty - y, tx - x) - yaw;
alpha = atan2(sin(alpha), cos(alpha));

delta = atan2(2 * params.L * sin(alpha), Ld);

delta_max = deg2rad(35);
delta = max(-delta_max, min(delta, delta_max));

end