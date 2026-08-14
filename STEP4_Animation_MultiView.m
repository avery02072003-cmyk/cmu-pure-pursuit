% STEP4_Animation_MultiView.m — 聯結車追蹤動畫（含即時局部路徑生成視覺化）
clear; clc; close all;
load('simulation_results.mat', 'results');

hist    = results.hist;
ts      = results.ts;
gps_wp  = results.gps_wp;
replan_log = results.replan_log;   % 每次 replan 的候選路徑歷史（供即時重畫）
refpath = results.refpath;
params  = results.params;

Nsim = length(ts);

% ---- 靜態背景：母路徑 + GPS waypoints ----
fig = figure('Color','k','Position',[100 100 1200 800]);
ax = axes('Parent', fig, 'Color','w'); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
ax.GridColor = [0.3 0.3 0.3]; ax.XColor='w'; ax.YColor='w';

plot(ax, refpath.x, refpath.y, 'w:', 'LineWidth', 0.6, 'DisplayName','Reference (母路徑)');

plot(ax, gps_wp(:,1), gps_wp(:,2), 'ko', 'MarkerFaceColor','k', ...
    'MarkerSize', 5, 'DisplayName','GPS Waypoints');

% ---- 即時局部候選路徑：每次 replan 都重新生成，用固定顏色代表「第幾條候選」----
colors = lines(params.N_paths);
h_cand = gobjects(1, params.N_paths);
for i = 1:params.N_paths
    h_cand(i) = plot(ax, NaN, NaN, '--', 'Color', colors(i,:), 'LineWidth', 1.2, ...
        'HandleVisibility', 'off');
end
h_cand_active = plot(ax, NaN, NaN, '-', 'Color', [1 1 1], 'LineWidth', 2.2, ...
    'DisplayName', 'Active Candidate（本次選中的路徑）');

% ---- 動態物件：tractor / trailer / hitch / 車輛外框 ----
h_tractor_trail = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName','Tractor 軌跡');
h_trailer_trail = plot(ax, NaN, NaN, 'g-', 'LineWidth', 1.2, 'DisplayName','Trailer 軌跡');

h_tractor_body = patch(ax, NaN, NaN, 'y', 'FaceAlpha', 0.8, 'EdgeColor','y', 'DisplayName','Tractor 車身');
h_trailer_body = patch(ax, NaN, NaN, 'm', 'FaceAlpha', 0.6, 'EdgeColor','m', 'DisplayName','Trailer 車身');
h_hitch_line   = plot(ax, NaN, NaN, 'r-', 'LineWidth', 1.5, 'DisplayName','Hitch 連結線');
h_kingpin      = plot(ax, NaN, NaN, 'ko', 'MarkerFaceColor','r', ...
    'MarkerSize', 6, 'DisplayName','Kingpin/Hitch');

h_title = title(ax, '', 'Color','w');
legend(ax, [h_cand_active, h_tractor_trail, h_trailer_trail, h_tractor_body, h_trailer_body, h_hitch_line], ...
    'Location','best', 'TextColor','w');
xlabel(ax,'X (m)','Color','w'); ylabel(ax,'Y (m)','Color','w');

skip = 1;   % 跳幀加速播放，可調整
playback_speed = 1.0;   % 1.0 = 正常速度；調小這個數字會播更慢，例如 0.3

replan_ptr = 0;   % 已經播放到 replan_log 的第幾筆

for k = 1:skip:Nsim
    % ---- 每追上一次新的 replan，就重畫候選路徑（即時路徑生成視覺化）----
    while replan_ptr < numel(replan_log) && replan_log(replan_ptr+1).step <= k
        replan_ptr = replan_ptr + 1;
        entry = replan_log(replan_ptr);
        for i = 1:params.N_paths
            if i <= numel(entry.cand_x) && ~isempty(entry.cand_x{i})
                set(h_cand(i), 'XData', entry.cand_x{i}, 'YData', entry.cand_y{i});
            else
                set(h_cand(i), 'XData', NaN, 'YData', NaN);
            end
        end
        ai = entry.active_idx;
        if ai >= 1 && ai <= numel(entry.cand_x) && ~isempty(entry.cand_x{ai})
            set(h_cand_active, 'XData', entry.cand_x{ai}, 'YData', entry.cand_y{ai});
        end
    end

    x0 = hist.x0(k); y0 = hist.y0(k); yaw0 = hist.yaw0(k);
    x1 = hist.x1(k); y1 = hist.y1(k); yaw1 = hist.yaw1(k);
    Hx = hist.Hx(k); Hy = hist.Hy(k);

    set(h_tractor_trail, 'XData', hist.x0(1:k), 'YData', hist.y0(1:k));
    set(h_trailer_trail, 'XData', hist.x1(1:k), 'YData', hist.y1(1:k));

    tractor_corners = local_to_world(x0, y0, yaw0, ...
    params.L1 + params.tractor_front_overhang, ...
    params.tractor_rear_overhang, ...
    params.tractor_width);

    trailer_corners = local_to_world(x1, y1, yaw1, ...
    params.L2 + params.trailer_front_overhang, ...
    params.trailer_rear_overhang, ...
    params.trailer_width);

    set(h_tractor_body, 'XData', tractor_corners(:,1), 'YData', tractor_corners(:,2));
    set(h_trailer_body, 'XData', trailer_corners(:,1), 'YData', trailer_corners(:,2));

    set(h_hitch_line, 'XData', [x0, Hx], 'YData', [y0, Hy]);
    set(h_kingpin, 'XData', Hx, 'YData', Hy);

    set(h_title, 'String', sprintf('t = %.2f s | Step %d/%d | Active Path %d | v = %.2f m/s', ...
        ts(k), k, Nsim, hist.active_idx(k), hist.v_cmd(k)));

    drawnow;
    pause(params.Ts / playback_speed);   % 依模擬時間步長控制真實播放節奏
end

function corners = local_to_world(x, y, yaw, L_front, L_rear, W)
    local = [ L_front  W/2;
             -L_rear   W/2;
             -L_rear  -W/2;
              L_front -W/2;
              L_front  W/2];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    world = (R * local')' + [x, y];
    corners = world;
end
