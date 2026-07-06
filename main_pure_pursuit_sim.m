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

for k = 1:Nsim
    [delta, idx_target, idx_near, ~] = pure_pursuit_controller(x, y, yaw, v, refpath, params, idx_prev);
    idx_prev = idx_near;

    x = x + v * cos(yaw) * params.Ts;
    y = y + v * sin(yaw) * params.Ts;
    yaw = yaw + v / params.L * tan(delta) * params.Ts;

    hist.x(k) = x;
    hist.y(k) = y;
    hist.yaw(k) = yaw;
    hist.delta(k) = delta;
    hist.idx_target(k) = idx_target;
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