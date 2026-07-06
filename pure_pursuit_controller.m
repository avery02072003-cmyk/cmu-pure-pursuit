function [delta, idx_target, idx_near, Ld] = pure_pursuit_controller(x, y, yaw, v, refpath, params, idx_prev)

Ld = params.Ld0 + params.kv * abs(v);
Ld = max(params.Ld_min, min(Ld, params.Ld_max));

dist_all = sqrt((refpath.x - x).^2 + (refpath.y - y).^2);

search_start = max(1, idx_prev);
[~, local_idx] = min(dist_all(search_start:end));
idx_near = search_start + local_idx - 1;

idx_target = idx_near;
for i = idx_near:length(refpath.x)
    if dist_all(i) >= Ld
        idx_target = i;
        break;
    end
end

tx = refpath.x(idx_target);
ty = refpath.y(idx_target);

alpha = atan2(ty - y, tx - x) - yaw;
alpha = atan2(sin(alpha), cos(alpha));

delta = atan2(2 * params.L * sin(alpha), Ld);
end