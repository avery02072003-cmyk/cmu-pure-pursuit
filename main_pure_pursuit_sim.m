clear; clc; close all;

load('reference_path.mat', 'refpath');

params.Ts = 0.05;
params.L  = 2.7;
params.Ld0 = 2.0;
params.kv = 0.3;
params.Ld_min = 2.0;
params.Ld_max = 8.0;

% ---- dynamic speed planning parameters ----
params.v_des = 6.0;         % desired cruise speed (m/s)
params.v_min = 1.0;         % minimum speed (m/s)
params.a_lat_max = 2.0;     % lateral acceleration limit (m/s^2)
params.a_acc_max = 1.0;     % acceleration limit (m/s^2)
params.a_dec_max = 1.5;     % deceleration limit (m/s^2)

% ---- estimate curvature from reference path ----
dx_ref = gradient(refpath.x);
dy_ref = gradient(refpath.y);
ds_ref = hypot(dx_ref, dy_ref);
ds_ref(ds_ref < 1e-6) = 1e-6;

phi_unwrap = unwrap(refpath.phi);
kappa_ref = gradient(phi_unwrap) ./ ds_ref;

refpath.kappa = kappa_ref;

% ---- curvature-based speed limit ----
v_curve = sqrt(params.a_lat_max ./ max(abs(refpath.kappa), 1e-4));
v_curve = min(v_curve, params.v_des);
v_curve = max(v_curve, params.v_min);

refpath.v_profile = v_curve(:);

x = refpath.x(1);
y = refpath.y(1);
yaw = refpath.phi(1);
v = refpath.v_profile(1);

Nsim = min(length(refpath.x), 3000);
idx_prev = 1;

hist.x = zeros(Nsim,1);
hist.y = zeros(Nsim,1);
hist.yaw = zeros(Nsim,1);
hist.v = zeros(Nsim,1);
hist.delta = zeros(Nsim,1);
hist.idx_target = zeros(Nsim,1);
hist.idx_near = zeros(Nsim,1);
hist.Ld = zeros(Nsim,1);
hist.alpha = zeros(Nsim,1);
hist.cte = zeros(Nsim,1);
hist.he = zeros(Nsim,1);
hist.kappa = zeros(Nsim,1);
hist.a_lat = zeros(Nsim,1);

for k = 1:Nsim
    [delta, idx_target, idx_near, Ld, alpha] = pure_pursuit_controller(x, y, yaw, v, refpath, params, idx_prev);
    idx_prev = idx_near;

    v_ref_now = refpath.v_profile(idx_near);

    if v_ref_now > v
        v = min(v + params.a_acc_max * params.Ts, v_ref_now);
    else
        v = max(v - params.a_dec_max * params.Ts, v_ref_now);
    end

    x_ref = refpath.x(idx_near);
    y_ref = refpath.y(idx_near);
    yaw_ref = refpath.phi(idx_near);

    dx = x - x_ref;
    dy = y - y_ref;

    cte = -sin(yaw_ref)*dx + cos(yaw_ref)*dy;
    he = atan2(sin(yaw - yaw_ref), cos(yaw - yaw_ref));

    kappa_now = tan(delta) / params.L;
    a_lat_now = v^2 * kappa_now;

    x = x + v * cos(yaw) * params.Ts;
    y = y + v * sin(yaw) * params.Ts;
    yaw = yaw + v / params.L * tan(delta) * params.Ts;
    yaw = atan2(sin(yaw), cos(yaw));

    hist.x(k) = x;
    hist.y(k) = y;
    hist.yaw(k) = yaw;
    hist.v(k) = v;
    hist.delta(k) = delta;
    hist.idx_target(k) = idx_target;
    hist.idx_near(k) = idx_near;
    hist.Ld(k) = Ld;
    hist.alpha(k) = alpha;
    hist.cte(k) = cte;
    hist.he(k) = he;
    hist.kappa(k) = kappa_now;
    hist.a_lat(k) = a_lat_now;
end

figure;
plot(refpath.x, refpath.y, 'r--', 'LineWidth', 1.5); hold on;
plot(hist.x, hist.y, 'b-', 'LineWidth', 1.5);
axis equal; grid on;
legend('Reference Path', 'Pure Pursuit Tracking');
title('Pure Pursuit Tracking Result');

figure;
plot(rad2deg(hist.delta), 'LineWidth', 1.2);
grid on;
xlabel('Step');
ylabel('Steering Angle (deg)');
title('Steering Command');

figure;
subplot(2,1,1);
plot(hist.cte, 'LineWidth', 1.2);
grid on;
ylabel('CTE (m)');
title('Tracking Errors');

subplot(2,1,2);
plot(rad2deg(hist.he), 'LineWidth', 1.2);
grid on;
xlabel('Step');
ylabel('Heading Error (deg)');

fprintf('CTE RMS = %.4f m\n', rms(hist.cte));
fprintf('Heading Error RMS = %.4f deg\n', rms(rad2deg(hist.he)));
fprintf('Max |delta| = %.4f deg\n', max(abs(rad2deg(hist.delta))));

figure;
subplot(2,1,1);
plot(refpath.v_profile, 'r--', 'LineWidth', 1.2); hold on;
plot(hist.v, 'b-', 'LineWidth', 1.2);
grid on;
legend('Reference Speed Profile', 'Actual Speed');
ylabel('Speed (m/s)');
title('Speed Profile');

subplot(2,1,2);
plot(hist.a_lat, 'LineWidth', 1.2); hold on;
yline(params.a_lat_max, 'r--');
yline(-params.a_lat_max, 'r--');
grid on;
xlabel('Step');
ylabel('Lateral Accel (m/s^2)');
title('Lateral Acceleration');

fprintf('CTE RMS = %.4f m\n', rms(hist.cte));
fprintf('Heading Error RMS = %.4f deg\n', rms(rad2deg(hist.he)));
fprintf('Max |delta| = %.4f deg\n', max(abs(rad2deg(hist.delta))));
fprintf('Speed range = [%.4f, %.4f] m/s\n', min(hist.v), max(hist.v));
fprintf('Max |a_lat| = %.4f m/s^2\n', max(abs(hist.a_lat)));