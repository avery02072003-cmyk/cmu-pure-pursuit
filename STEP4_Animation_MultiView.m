% STEP4_ETS2_Animation_MultiView.m — 聯結車追蹤動畫
clear; clc; close all;
load('simulation_results.mat', 'results');

hist    = results.hist;
ts      = results.ts;
gps_wp  = results.gps_wp;
path_candidates = results.path_candidates;
refpath = results.refpath;
params  = results.params;

Nsim = length(ts);

% ---- 靜態背景：母路徑 + 候選路徑 + GPS waypoints ----
fig = figure('Color','k','Position',[100 100 1200 800]);
ax = axes('Parent', fig, 'Color','w'); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
ax.GridColor = [0.3 0.3 0.3]; ax.XColor='w'; ax.YColor='w';

plot(ax, refpath.x, refpath.y, 'w:', 'LineWidth', 0.6, 'DisplayName','Reference (母路徑)');

colors = lines(params.N_paths);
for i = 1:params.N_paths
    if ~isempty(path_candidates{i})
        plot(ax, path_candidates{i}.x, path_candidates{i}.y, '--', ...
            'Color', colors(i,:), 'LineWidth', 1.0, 'HandleVisibility','off');
    end
end

plot(ax, gps_wp(:,1), gps_wp(:,2), 'ko', 'MarkerFaceColor','k', ...
    'MarkerSize', 5, 'DisplayName','GPS Waypoints');

% ---- 動態物件：tractor / trailer / hitch / 車輛外框 ----
h_tractor_trail = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName','Tractor 軌跡');
h_trailer_trail = plot(ax, NaN, NaN, 'g-', 'LineWidth', 1.2, 'DisplayName','Trailer 軌跡');

h_tractor_body = patch(ax, NaN, NaN, 'y', 'FaceAlpha', 0.8, 'EdgeColor','y', 'DisplayName','Tractor 車身');
h_trailer_body = patch(ax, NaN, NaN, 'm', 'FaceAlpha', 0.6, 'EdgeColor','m', 'DisplayName','Trailer 車身');
h_hitch_line   = plot(ax, NaN, NaN, 'r-', 'LineWidth', 1.5, 'DisplayName','Hitch 連結線');

h_title = title(ax, '', 'Color','w');
legend(ax, [h_tractor_trail, h_trailer_trail, h_tractor_body, h_trailer_body, h_hitch_line], ...
    'Location','best', 'TextColor','w');
xlabel(ax,'X (m)','Color','w'); ylabel(ax,'Y (m)','Color','w');

% ---- 車輛外框尺寸（簡化為矩形示意）----
L_tractor = params.L1; W_tractor = 2.2;
L_trailer = params.L2; W_trailer = 2.4;

skip = 1;   % 跳幀加速播放，可調整
playback_speed = 1.0;   % 1.0 = 正常速度；調小這個數字會播更慢，例如 0.3

for k = 1:skip:Nsim
    x0 = hist.x0(k); y0 = hist.y0(k); yaw0 = hist.yaw0(k);
    x1 = hist.x1(k); y1 = hist.y1(k); yaw1 = hist.yaw1(k);
    Hx = hist.Hx(k); Hy = hist.Hy(k);

    set(h_tractor_trail, 'XData', hist.x0(1:k), 'YData', hist.y0(1:k));
    set(h_trailer_trail, 'XData', hist.x1(1:k), 'YData', hist.y1(1:k));

    tractor_corners = local_to_world(x0, y0, yaw0, L_tractor, W_tractor);
    trailer_corners = local_to_world(x1, y1, yaw1, L_trailer, W_trailer);
    set(h_tractor_body, 'XData', tractor_corners(:,1), 'YData', tractor_corners(:,2));
    set(h_trailer_body, 'XData', trailer_corners(:,1), 'YData', trailer_corners(:,2));

    set(h_hitch_line, 'XData', [x0, Hx, x1], 'YData', [y0, Hy, y1]);

    set(h_title, 'String', sprintf('t = %.2f s | Step %d/%d | Active Path %d | v = %.2f m/s', ...
        ts(k), k, Nsim, hist.active_idx(k), hist.v_cmd(k)));

    drawnow;
    pause(params.Ts / playback_speed);   % 依模擬時間步長控制真實播放節奏
end

function corners = local_to_world(x, y, yaw, L, W)
    local = [ L/2  W/2; -L/2  W/2; -L/2 -W/2;  L/2 -W/2;  L/2  W/2];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    world = (R * local')' + [x, y];
    corners = world;
end