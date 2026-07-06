clear; clc; close all;

load('reference_path.mat', 'refpath');

params.Ts = 0.05;
params.L  = 2.7;
params.Ld0 = 2.0;
params.kv = 0.3;
params.Ld_min = 2.0;
params.Ld_max = 8.0;

x = refpath.x(1);
y = refpath.y(1);
yaw = refpath.phi(1);
v = 3.0;

Nsim = min(length(refpath.x), 3000);
idx_prev = 1;

hist.x = zeros(Nsim,1);
hist.y = zeros(Nsim,1);
hist.yaw = zeros(Nsim,1);
hist.delta = zeros(Nsim,1);
hist.idx_target = zeros(Nsim,1);
hist.idx_near = zeros(Nsim,1);
hist.Ld = zeros(Nsim,1);
hist.alpha = zeros(Nsim,1);
hist.cte = zeros(Nsim,1);
hist.he = zeros(Nsim,1);

for k = 1:Nsim
    [delta, idx_target, idx_near, Ld, alpha] = pure_pursuit_controller(x, y, yaw, v, refpath, params, idx_prev);
    idx_prev = idx_near;

    x_ref = refpath.x(idx_near);
    y_ref = refpath.y(idx_near);
    yaw_ref = refpath.phi(idx_near);

    dx = x - x_ref;
    dy = y - y_ref;

    cte = -sin(yaw_ref)*dx + cos(yaw_ref)*dy;
    he = atan2(sin(yaw - yaw_ref), cos(yaw - yaw_ref));

    x = x + v * cos(yaw) * params.Ts;
    y = y + v * sin(yaw) * params.Ts;
    yaw = yaw + v / params.L * tan(delta) * params.Ts;
    yaw = atan2(sin(yaw), cos(yaw));

    hist.x(k) = x;
    hist.y(k) = y;
    hist.yaw(k) = yaw;
    hist.delta(k) = delta;
    hist.idx_target(k) = idx_target;
    hist.idx_near(k) = idx_near;
    hist.Ld(k) = Ld;
    hist.alpha(k) = alpha;
    hist.cte(k) = cte;
    hist.he(k) = he;
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