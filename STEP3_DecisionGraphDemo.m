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
h_title = title(ax, '', 'Color', 'w', 'FontSize', 12);
legend(ax, 'Location', 'bestoutside', 'TextColor', 'w');
xlabel(ax, 'X (m)', 'Color', 'w'); ylabel(ax, 'Y (m)', 'Color', 'w');

view_radius = 40;   % 每一步自動縮放到車輛周圍 ±40m 的範圍，方便看清楚分支細節
pause_sec = 1.5;    % 每個決策點之間的展示停留時間

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

    % ---- 依車道序列挑選這一步要走的分支，把 state 移動到該分支終點 ----
    chosen = branches([branches.to_lane_id] == next_lane_id);
    if isempty(chosen)
        error('STEP3_DecisionGraphDemo: 車道序列第 %d→%d 步不合法（不在允許的分支範圍內）', ...
            current_lane_id, next_lane_id);
    end
    chosen = chosen(1);

    state = [chosen.x(end), chosen.y(end), chosen.phi(end), params.v_des];
    current_lane_id = next_lane_id;
end

% ---- 最後一個決策點也畫出來，讓展示停在完整的畫面上 ----
branches = generate_decision_branches(state, refpath, params, current_lane_id);
h = update_decision_graph_plot(h, state, branches);
set(ax, 'XLim', [state(1)-view_radius, state(1)+view_radius], ...
        'YLim', [state(2)-view_radius, state(2)+view_radius]);
set(h_title, 'String', sprintf('決策點 %d/%d（結束）｜目前車道 %+d ｜ v = %.2f m/s', ...
    numel(demo_lane_sequence), numel(demo_lane_sequence)-1, current_lane_id, state(4)));
fprintf('=== 展示結束 ===\n');
