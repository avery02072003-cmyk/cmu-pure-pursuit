% =========================================================================
% 檔案名稱: STEP3_DecisionGraphDemo.m
%
% 功能：離散車道決策分支路徑視覺化的獨立展示腳本。跟 STEP2_MultiPathGen.m
%       一樣，是「非必要執行步驟，純粹用來視覺化確認」的預覽腳本
%       （不影響、也不需要 main_pure_pursuit_sim.m 先跑過），只需要
%       vehicle_params.mat + reference_path.mat 就能執行。
%
%       畫面呈現：黑點＝車輛目前選定的位置；從黑點分支出 2~3 條候選
%       曲線（依所在車道決定 2 或 3 條，見 generate_decision_branches.m
%       檔頭的分支規則）；每條分支末端用 X 標記代表「選了這條分支之後、
%       下一次重新決策的位置」。
%
% 為什麼用「腳本自行排定的車道序列」而不是重播 main_pure_pursuit_sim.m
% 的真實模擬記錄：
%       現有模擬在沒有障礙物時，select_best_path.m 的評分機制永遠選中
%       置中候選路徑（CTE 最小），車輛整趟模擬都不會換道。如果重播
%       simulation_results.mat 的真實記錄，車輛的車道身分（用
%       estimate_current_lane_id.m 算出來）會全程都是 0（中間車道），
%       畫面就會一直是「3 分支」情境，永遠不會出現你想看的「2 分支」
%       （左/右車道）情境——這樣不但沒展示到功能，還可能讓人誤以為
%       換道分支的程式碼有問題。改成腳本自己排一個會經過所有情境的
%       車道序列，確保 3 分支/2 分支、直行/換道、左變道/右變道全部
%       都會被展示到。
%
% 車道序列 demo_lane_sequence = [0,1,1,0,0,-1,-1,0]，依序代表：
%       0→1  （中間車道，選「變換到左車道」分支）
%       1→1  （左車道，選「直行」分支）
%       1→0  （左車道，選「變回中間車道」分支）
%       0→0  （中間車道，選「直行」分支）
%       0→-1 （中間車道，選「變換到右車道」分支）
%       -1→-1（右車道，選「直行」分支）
%       -1→0 （右車道，選「變回中間車道」分支）
%       涵蓋 3 分支情境（中間車道，出現 3 次）、2 分支情境（左右車道，
%       各出現 2 次）、以及全部 3 種分支類型（straight/change_left/
%       change_right）。
%
% 動畫播放（每一步選定分支之後，不再瞬間跳到終點）：
%       黑點會沿著選中的分支曲線移動，同時用一條黃色短線標示當下航向，
%       讓換道時的航向如何隨弧長平滑轉變（lane_change_offset_profile.m
%       的 dphi_correction，見該檔案檔頭推導）可以直接用肉眼確認是否
%       平滑、有沒有在 segment 交界處出現轉向不連續的「抽動」。沒被選中
%       的其餘分支曲線在這段動畫期間維持顯示，方便對照「這是從哪些選項
%       中選出來的」。
%
%       播放節奏跟 STEP4_Animation_MultiView.m 用同一套「固定播放時間格
%       ＋ playback_speed 縮放」模型（該檔案第 196-197 行的 skip/
%       playback_speed），而不是逐點播放 chosen.x/y/phi 本身：chosen.x/
%       y/phi 是 stitch_local_path.m 曲線擬合內部輸出的密集點（動輒
%       2000+ 點，密度由 Newton-Raphson 擬合品質決定，不是依播放需求
%       決定），若逐點呼叫 drawnow/pause，即使每次要求的等待時間很短，
%       數千次呼叫本身的固定開銷疊加起來就會讓畫面明顯卡頓變慢——這是
%       先前版本「看起來非常慢」的真正原因，不是 pause() 時間算錯。
%       修正方式：先用 resample_window_by_arclength.m 依弧長等距抽稀成
%       「播放張數」（由分支總長度 / (v*anim_dt) 決定，車速越快、單一
%       播放格代表的距離越長，張數越少），只對這些抽稀後的張數逐張呼叫
%       drawnow/pause，張數跟 STEP4 的畫面更新張數同一數量級（幾十張），
%       不影響 chosen.x/y/phi 本身的擬合密度。
% =========================================================================

clear; clc; close all;
load('vehicle_params.mat', 'params');
load('reference_path.mat', 'refpath');

% ---- 選一個離路線接縫夠遠的起點，避免示範一開始就撞到環形路線的頭尾接縫 ----
start_idx = round(length(refpath.x) * 0.15);
state = [refpath.x(start_idx), refpath.y(start_idx), refpath.phi(start_idx), params.v_des];
current_lane_id = 0;

% ---- 車道序列（見檔頭說明），依序走過去 ----
demo_lane_sequence = [0, 1, 1, 0, 0, -1, -1, 0];

% ---- 繪圖初始化 ----
fig = figure('Color', 'k', 'Position', [100 100 1100 850]);
ax = axes('Parent', fig, 'Color', 'k'); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
ax.GridColor = [0.35 0.35 0.35]; ax.XColor = 'w'; ax.YColor = 'w';

plot(ax, refpath.x, refpath.y, 'w:', 'LineWidth', 0.5, 'DisplayName', 'Reference (母路徑/車道中心線)');

% ---- 畫出左/中/右 3 條車道的邊界線，讓分支曲線是否真的落在車道寬度內、
% 換道分支是否真的到達鄰車道，都可以直接肉眼對照確認（跟 generate_
% decision_branches.m 分支終點的數值驗證互相印證：已用最近點投影量過，
% 每條換道分支的終點橫向偏移跟目標車道中心的誤差是 0.000m）----
% 航向先做移動平均平滑化再算法線方向，理由跟 STEP4_Animation_MultiView.m
% 車道邊界線的做法相同：密集點上直接用 gradient() 重新估計切線方向，
% 微小位置雜訊會被放大成看起來抖動的邊界線。
lane_half = params.lane_width / 2;
phi_unwrap = unwrap(refpath.phi);
smooth_win = 51;
smooth_kernel = ones(1, smooth_win) / smooth_win;
phi_smooth = conv([phi_unwrap(1)*ones((smooth_win-1)/2,1); phi_unwrap(:); phi_unwrap(end)*ones((smooth_win-1)/2,1)], ...
    smooth_kernel, 'valid');
nx = -sin(phi_smooth); ny = cos(phi_smooth);
for k = [-1.5, -0.5, 0.5, 1.5]   % 4 條邊界線，圍出左/中/右 3 條車道
    off_k = k * params.lane_width;
    b_x = refpath.x + off_k*nx(:);
    b_y = refpath.y + off_k*ny(:);
    if abs(k) == 0.5
        plot(ax, b_x, b_y, '-.', 'Color', [0.9 0.9 0.3], 'LineWidth', 1.4, 'HandleVisibility', 'off');   % 本車道邊界，較亮
    else
        plot(ax, b_x, b_y, '-.', 'Color', [0.55 0.55 0.18], 'LineWidth', 0.9, 'HandleVisibility', 'off'); % 鄰車道外側邊界，較暗
    end
end
plot(ax, NaN, NaN, '-.', 'Color', [0.9 0.9 0.3], 'LineWidth', 1.4, 'DisplayName', sprintf('車道邊界（每條車道寬 %.1fm）', params.lane_width));

h = init_decision_graph_plot(ax);

% ---- 航向指示線：動畫播放時跟著黑點移動，標示當下航向方向 ----
% （黃色純粹為了在深色背景、跟分支顏色都對比明顯，不隨分支 type 變色）
h_heading = plot(ax, NaN, NaN, '-', 'Color', [1 1 0], 'LineWidth', 2.5, 'HandleVisibility', 'off');
heading_len = 3;   % [m] 航向指示線長度，純視覺化用途，不代表任何物理尺寸

h_title = title(ax, '', 'Color', 'w', 'FontSize', 12);
legend(ax, 'Location', 'bestoutside', 'TextColor', 'w');
xlabel(ax, 'X (m)', 'Color', 'w'); ylabel(ax, 'Y (m)', 'Color', 'w');

view_radius = 40;   % 每一步自動縮放到車輛周圍 ±40m 的範圍，方便看清楚分支細節
pause_sec = 1.5;    % 每個決策點之間的展示停留時間（顯示分支選項時）

% ---- 分支曲線移動動畫的播放節奏（跟 STEP4_Animation_MultiView.m 同一套模型）----
% anim_dt 是「一張播放畫面代表多少真實秒數」，純粹是這個展示腳本的播放
% 節奏，跟控制迴圈的 params.Ts（0.02~0.05s，另一回事）無關，也不能直接
% 拿 params.Ts 用——那樣算出來的播放張數（同一數量級的問題）一樣會太多。
anim_dt = 0.1;         % [s] 每張播放畫面代表的時間
playback_speed = 2.0;  % 1.0 = 正常速度；調小這個數字會播更慢，例如 0.5

type_name_zh = struct('straight', '直行', 'change_left', '變換到左車道', 'change_right', '變換到右車道');

fprintf('=== STEP3: 離散車道決策分支路徑展示 ===\n');
for step = 1:numel(demo_lane_sequence)-1
    next_lane_id = demo_lane_sequence(step+1);

    branches = generate_decision_branches(state, refpath, params, current_lane_id);

    fprintf('第 %d 步：目前車道=%+d，v=%.2fm/s，本次生成 %d 條分支：', ...
        step, current_lane_id, state(4), numel(branches));
    for i = 1:numel(branches)
        fprintf(' [%s→車道%+d]', type_name_zh.(branches(i).type), branches(i).to_lane_id);
    end
    fprintf('\n');

    h = update_decision_graph_plot(h, state, branches);
    set(ax, 'XLim', [state(1)-view_radius, state(1)+view_radius], ...
            'YLim', [state(2)-view_radius, state(2)+view_radius]);
    set(h_title, 'String', sprintf('決策點 %d/%d ｜目前車道 %+d ｜ v = %.2f m/s', ...
        step, numel(demo_lane_sequence)-1, current_lane_id, state(4)));
    drawnow;
    pause(pause_sec);

    % ---- 依車道序列挑選這一步要走的分支 ----
    chosen = branches([branches.to_lane_id] == next_lane_id);
    if isempty(chosen)
        error('STEP3_DecisionGraphDemo: 車道序列第 %d→%d 步不合法（不在允許的分支範圍內）', ...
            current_lane_id, next_lane_id);
    end
    chosen = chosen(1);

    % ---- 動畫播放：黑點真正沿著選中的分支曲線移動，不是瞬間跳到終點 ----
    % 先依弧長把 chosen.x/y/phi（曲線擬合內部密集輸出）抽稀成播放張數，
    % 張數由「分支總長度 / (目前車速 * anim_dt)」決定，車速越快單張代表
    % 的距離越長、張數越少（跟 STEP4 依 Ts 決定播放張數同一套道理）。
    % phi 先 unwrap 再內插、內插完再包回 (-pi, pi]，避免航向剛好跨過
    % ±180° 邊界時內插出錯誤的跳變（純數值處理細節，不影響實際航向）。
    % （其餘未選中的分支曲線這段期間維持顯示，見檔頭說明）
    s_chosen = [0; cumsum(hypot(diff(chosen.x), diff(chosen.y)))];
    total_dist = s_chosen(end);
    frame_spacing = max(state(4), 0.1) * anim_dt;
    n_frames = max(2, round(total_dist / frame_spacing) + 1);
    s_frames = linspace(0, total_dist, n_frames)';
    [fx, fy, fphi_unwrapped] = resample_window_by_arclength(chosen.x, chosen.y, unwrap(chosen.phi), s_chosen, s_frames);
    fphi = atan2(sin(fphi_unwrapped), cos(fphi_unwrapped));

    for j = 1:n_frames
        set(h.dot, 'XData', fx(j), 'YData', fy(j));
        set(h_heading, 'XData', [fx(j), fx(j) + heading_len*cos(fphi(j))], ...
                       'YData', [fy(j), fy(j) + heading_len*sin(fphi(j))]);
        set(ax, 'XLim', [fx(j)-view_radius, fx(j)+view_radius], ...
                'YLim', [fy(j)-view_radius, fy(j)+view_radius]);
        set(h_title, 'String', sprintf('第 %d 步：%s（往車道 %+d）移動中 %d/%d', ...
            step, type_name_zh.(chosen.type), chosen.to_lane_id, j, n_frames));
        drawnow;
        pause(anim_dt / playback_speed);
    end

    state = [chosen.x(end), chosen.y(end), chosen.phi(end), params.v_des];
    current_lane_id = next_lane_id;
end

% ---- 走完全部 7 個決策點後，在最終停留位置額外多畫一次分支圖 ----
% 這一格不是序列裡的第 8 個決策點（demo_lane_sequence 只有 7 次轉換，
% 決策點編號到 7 就結束了），純粹是讓展示停留在「終點位置也能看到接下來
% 有哪些選項」的完整畫面，標題不用「X/Y」分數格式，避免看起來像多出一個
% 對不起來的決策點編號。
branches = generate_decision_branches(state, refpath, params, current_lane_id);
h = update_decision_graph_plot(h, state, branches);
set(ax, 'XLim', [state(1)-view_radius, state(1)+view_radius], ...
        'YLim', [state(2)-view_radius, state(2)+view_radius]);
set(h_title, 'String', sprintf('展示結束（共 %d 個決策點）｜最終位置車道 %+d ｜ v = %.2f m/s', ...
    numel(demo_lane_sequence)-1, current_lane_id, state(4)));
fprintf('=== 展示結束（共 %d 個決策點） ===\n', numel(demo_lane_sequence)-1);
