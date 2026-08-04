% STEP2_MultiPathGen.m — 多路徑預覽
clear; clc; close all;
load('vehicle_params.mat', 'params');
load('reference_path.mat', 'refpath');

stride = 50;
gps_wp = [refpath.x(1:stride:end), refpath.y(1:stride:end)];

path_candidates = my_multi_path(gps_wp, params.N_paths, params);

figure; hold on; axis equal; grid on;
colors = lines(params.N_paths);
for i = 1:params.N_paths
    p = path_candidates{i};
    if ~isempty(p) && isfield(p, 'x')
        plot(p.x, p.y, '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
             'DisplayName', sprintf('Path %d (offset=%.1fm)', i, ...
             params.lane_width/2 * (2*(i-1)/(params.N_paths-1)-1)));
    end
end
plot(gps_wp(:,1), gps_wp(:,2), 'k+', 'MarkerSize', 10, 'DisplayName', 'GPS Waypoints');
legend; title('Multi-Path Candidates Preview');
xlabel('X (m)'); ylabel('Y (m)');