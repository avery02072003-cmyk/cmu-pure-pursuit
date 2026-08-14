% =========================================================================
% 檔案名稱: STEP4_Animation_MultiView.m
%
% 功能：讀取 main_pure_pursuit_sim.m 存出的 simulation_results.mat，
%       播放整趟模擬的動畫：拖車頭/貨櫃車身、行駛軌跡、鉸接連結線，
%       並且即時重畫「每次 replan 當下生成的候選路徑」，讓即時局部路徑
%       生成的過程可以用肉眼直接看到（不是模擬開始前就畫死的固定背景）。
%
% 使用方式：
%   須先執行 main_pure_pursuit_sim.m 產生 simulation_results.mat，本腳本
%   才有資料可以播放。開啟 MATLAB 直接執行本檔案即可看到動畫視窗。
%
% 即時候選路徑重畫的原理（這是本次新增的功能）：
%   main_pure_pursuit_sim.m 每次 replan（每 T_replan 步，約 0.5 秒）都會
%   把「這次呼叫 generate_local_paths() 生成的 5 條候選路徑座標」記錄進
%   results.replan_log（每一筆含 step、active_idx、cand_x、cand_y）。
%   本腳本用 replan_ptr 這個游標追蹤「動畫播放進度 k」跟「replan_log
%   第幾筆」的對應關係：每當動畫播放的步數 k 超過 replan_log 下一筆
%   紀錄的 step，就把畫面上代表候選路徑的線條（h_cand 陣列）更新成
%   那一筆記錄的座標。這樣播放時，觀眾會看到候選路徑（虛線，5 種
%   顏色）跟被選中的路徑（白色粗實線）隨著車輛前進、每 0.5 秒重新
%   長出來一次，直接對應 generate_local_paths.m 檔頭說明的「即時、
%   以車輛當下位置為錨點重新生成」這件事。
%
%   （這次新增之前，舊版腳本只在動畫開始「前」畫一次候選路徑當靜態
%   背景，而且當時 simulation_results.mat 也只存了整趟模擬「最後一次」
%   replan 的候選路徑，兩個因素疊加，導致候選路徑在畫面上完全不會
%   隨車輛移動更新，看起來像是固定不變、而且長度只有一小段局部窗口，
%   容易讓人誤會即時生成沒有在運作。）
%
% 固定車道邊界線（這次新增）：
%   左右車道邊界＝母路徑（中心線）沿法線方向各平移 lane_width/2，
%   直接呼叫既有的 shift_waypoints_lateral.m（跟候選路徑側向平移用
%   同一個函式），確保邊界線的幾何跟候選路徑產生時所依據的「可用道路
%   寬度」定義完全一致，不會另外用一套不同的公式算出不一致的邊界。
%   這條邊界線是整條賽道固定不變的（母路徑本身沒有隨模擬時間變化），
%   所以只需要在動畫開始前畫一次，不用像候選路徑那樣每幀更新。
%
%   另外加上即時「貨櫃是否壓線」文字讀數：每一幀都用貨櫃後軸位置
%   (x1,y1) 對母路徑做最近點投影，算出貨櫃中心到車道中心線的橫向
%   偏移量，再加上貨櫃半寬（trailer_width/2）估計貨櫃「外側車身邊緣」
%   到車道邊界的距離 —— 這比只看貨櫃中心點更準確，因為真正判斷「壓線」
%   要看車身輪廓有沒有超出邊界，不是只看軸心。距離為正代表還在車道內、
%   為負代表已經壓出車道邊界，文字顏色會跟著綠/紅切換。
% =========================================================================

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
% 注意：座標軸背景改成黑色（原本是白色 'w'，但格線/座標軸文字/母路徑
% 都是設計成白色系配色，配白色背景等於直接把自己畫不見——這是先前
% 車道邊界線「顏色太淡」問題的根本原因：不是邊界線顏色選得不夠深，
% 是整體配色方案原本就是設計給深色背景用的。改成黑色背景後，白色
% 母路徑、淺灰車道邊界線都能維持原本設計的高對比可見度，不需要
% 額外把每個元素的顏色都重新挑過。
fig = figure('Color','k','Position',[100 100 1200 800]);
ax = axes('Parent', fig, 'Color','k'); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
ax.GridColor = [0.35 0.35 0.35]; ax.XColor='w'; ax.YColor='w';

plot(ax, refpath.x, refpath.y, 'w:', 'LineWidth', 0.6, 'DisplayName','Reference (母路徑/車道中心線)');

% GPS waypoints 改成暗灰、縮小標記：原本用亮黑點、間距只有 ~0.31m，
% 放大到車輛附近的局部視角（例如手動縮放到 20~30m 範圍）時，密集的
% 圓點會連成一串「一顆一顆拼起來」的視覺效果，容易被誤認成路徑本身
% 是分段拼接、不平滑。調暗、縮小後只在整體俯視時提供路線參考，
% 不會在放大檢視局部路段時喧賓奪主。
plot(ax, gps_wp(:,1), gps_wp(:,2), 'o', 'Color', [0.45 0.45 0.45], ...
    'MarkerFaceColor', [0.45 0.45 0.45], 'MarkerSize', 2, 'DisplayName','GPS Waypoints');

% ---- 固定車道邊界線：本車道 + 左右各 n_side_lanes 條鄰車道 ----
% 這裡刻意不直接呼叫 shift_waypoints_lateral.m（它內部用 gradient() 對
% 密集點重新估計切線方向），而是先把母路徑既有的 refpath.phi 做一次
% 移動平均平滑化，再用平滑後的航向算法線方向：純位置資料在密集取樣
% 下本身已經很平滑，但「方向」是位置的一階導數，任何微小的位置雜訊
% 都會被放大，導致偏移出來的邊界線看起來比中心線本身更容易有細碎的
% 抖動（這正是「車道線像一段一段拼起來」的視覺成因）。平滑視窗跟
% compute_path_curvature.m 用的手法一致，只是這裡只用於本檔案的
% 視覺呈現，不影響任何實際路徑生成/追蹤邏輯。
%
% 邊界線數量：候選路徑側向範圍涵蓋本車道 + 左右各 n_side_lanes 條鄰車道
% （跟 generate_local_paths.m 產生候選路徑用的範圍完全一致），所以要畫
% 出「本車道邊界」（較粗、較亮）跟「鄰車道外側邊界」（較細、較暗）幾組
% 線，才能讓你看出候選路徑目前落在哪一條車道裡。
n_side_lanes = 0;
if isfield(params, 'n_side_lanes'), n_side_lanes = params.n_side_lanes; end
lane_half = params.lane_width / 2;
phi_unwrap = unwrap(refpath.phi);
smooth_win = 51;
smooth_kernel = ones(1, smooth_win) / smooth_win;
phi_smooth = conv([phi_unwrap(1)*ones((smooth_win-1)/2,1); phi_unwrap(:); phi_unwrap(end)*ones((smooth_win-1)/2,1)], ...
    smooth_kernel, 'valid');
nx = -sin(phi_smooth); ny = cos(phi_smooth);

% 本車道邊界（offset = ±lane_half）：亮黃色點劃線
lane_left  = [refpath.x + lane_half*nx(:), refpath.y + lane_half*ny(:)];
lane_right = [refpath.x - lane_half*nx(:), refpath.y - lane_half*ny(:)];
h_lane_boundary = plot(ax, lane_left(:,1),  lane_left(:,2),  '-.', 'Color', [0.9 0.9 0.3], 'LineWidth', 1.6, ...
    'DisplayName', sprintf('本車道邊界（寬 %.1f m）', params.lane_width));
plot(ax, lane_right(:,1), lane_right(:,2), '-.', 'Color', [0.9 0.9 0.3], 'LineWidth', 1.6, ...
    'HandleVisibility', 'off');

% 鄰車道外側邊界（offset = ±(k+0.5)*lane_width，k=1..n_side_lanes）：暗黃色點劃線
h_neighbor_boundary = gobjects(0);
for k = 1:n_side_lanes
    off_k = (k + 0.5) * params.lane_width;
    nb_left  = [refpath.x + off_k*nx(:), refpath.y + off_k*ny(:)];
    nb_right = [refpath.x - off_k*nx(:), refpath.y - off_k*ny(:)];
    hL = plot(ax, nb_left(:,1),  nb_left(:,2),  '-.', 'Color', [0.55 0.55 0.18], 'LineWidth', 1.0, ...
        'HandleVisibility', 'off');
    plot(ax, nb_right(:,1), nb_right(:,2), '-.', 'Color', [0.55 0.55 0.18], 'LineWidth', 1.0, ...
        'HandleVisibility', 'off');
    h_neighbor_boundary(end+1) = hL; %#ok<SAGROW>
end
if n_side_lanes > 0
    set(h_neighbor_boundary(1), 'HandleVisibility', 'on', ...
        'DisplayName', sprintf('鄰車道外側邊界（左右各 %d 條）', n_side_lanes));
end

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
h_kingpin      = plot(ax, NaN, NaN, 'o', 'MarkerEdgeColor','w', 'MarkerFaceColor','r', ...
    'MarkerSize', 6, 'DisplayName','Kingpin/Hitch');

% ---- 即時「貨櫃壓線」狀態讀數（左上角文字方塊，隨貨櫃位置即時更新）----
h_lane_status = text(ax, 0.015, 0.985, '', 'Units','normalized', ...
    'VerticalAlignment','top', 'FontSize', 11, 'FontWeight','bold', ...
    'BackgroundColor', [0 0 0 0.55], 'Margin', 4, 'Color', 'g');

h_title = title(ax, '', 'Color','w');
legend_handles = [h_lane_boundary, h_cand_active, h_tractor_trail, h_trailer_trail, h_tractor_body, h_trailer_body, h_hitch_line];
if n_side_lanes > 0
    legend_handles = [legend_handles, h_neighbor_boundary(1)];
end
legend(ax, legend_handles, 'Location','best', 'TextColor','w');
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

    % ---- 即時計算貨櫃是否壓線 ----
    % 1. 貨櫃後軸 (x1,y1) 對母路徑（車道中心線）做最近點投影，
    %    算法跟 pure_pursuit_controller.m / select_best_path.m 算 CTE 用的
    %    「投影到路徑法線方向」公式相同
    dist2_trailer = (refpath.x - x1).^2 + (refpath.y - y1).^2;
    [~, idx_trailer] = min(dist2_trailer);
    yaw_ref_trailer = refpath.phi(idx_trailer);
    dxr = x1 - refpath.x(idx_trailer); dyr = y1 - refpath.y(idx_trailer);
    trailer_lat_offset = -sin(yaw_ref_trailer)*dxr + cos(yaw_ref_trailer)*dyr;   % 貨櫃中心到車道中心線的橫向偏移（正=左偏，負=右偏）

    % 2. 加上貨櫃半寬，估計「貨櫃外側車身邊緣」到車道邊界的餘裕距離
    %    （看車身輪廓有沒有超出邊界，而不是只看軸心，才是真正的壓線判斷）
    margin_to_boundary = lane_half - (abs(trailer_lat_offset) + params.trailer_width/2);

    if margin_to_boundary >= 0
        status_str = sprintf('貨櫃在車道內，距邊界尚有 %.2f m（車道寬 %.1f m）', margin_to_boundary, params.lane_width);
        status_color = [0.2 0.9 0.2];
    else
        status_str = sprintf('⚠ 貨櫃已壓線，超出邊界 %.2f m（車道寬 %.1f m）', -margin_to_boundary, params.lane_width);
        status_color = [1.0 0.2 0.2];
    end
    set(h_lane_status, 'String', status_str, 'Color', status_color);

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

% -------------------------------------------------------------------------
% 子函式：local_to_world — 把「車身矩形的四個角點」從車輛局部座標系
% 轉換到世界座標系，供 patch() 畫出隨車輛移動/旋轉的車身矩形。
% -------------------------------------------------------------------------
% 局部座標系定義：原點在車輛的參考點（拖車頭後軸中心 或 貨櫃後軸中心），
% x 軸正方向＝車輛前進方向。車身矩形四角在局部座標系下的座標，就是
% 「往前 L_front、往後 L_rear、車寬一半 W/2」組成的矩形四個頂點：
%     (L_front, W/2), (-L_rear, W/2), (-L_rear, -W/2), (L_front, -W/2)
% 再多重複第一個點一次（矩形第5個點=第1個點），讓 patch/plot 畫出來
% 是封閉的矩形而不是缺一邊。
%
% 轉換到世界座標系：先用標準二維旋轉矩陣 R(yaw) 把矩形旋轉到車輛
% 目前航向，再平移到車輛目前位置 (x,y)：
%     world = R(yaw) * local + [x,y]
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
