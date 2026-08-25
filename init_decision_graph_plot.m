% =========================================================================
% 檔案名稱: init_decision_graph_plot.m
%
% 功能：建立決策分支圖的所有圖形物件 handle（黑點、最多 3 條分支曲線、
%       每條分支末端的 X 標記），初始都是 NaN 佔位，之後每一輪決策由
%       update_decision_graph_plot.m 用 set() 更新座標與樣式。
%
%       這個「init 建立佔位 handle、update 用 set() 更新」的模式，
%       跟 STEP4_Animation_MultiView.m 動畫已經在用的模式一致（該檔案
%       的 h_cand / h_tractor_trail 等也是同樣做法），刻意保持風格
%       一致，方便之後如果要把決策圖疊加進即時動畫，可以直接沿用。
%
% 分支類型與顏色（跟 generate_decision_branches.m 的 .type 對應）：
%       straight     淺灰色（直行，最「平常」的選擇，用最不顯眼的顏色）
%       change_left  青色（變換到左車道）
%       change_right 橘色（變換到右車道）
%
% 輸入：
%   ax : 要畫圖的座標軸 handle
%
% 輸出：
%   h : struct，包含所有圖形物件 handle：
%       .dot                目前位置的黑點
%       .branch_line(1:3)   最多 3 條分支曲線（用不到的槽位以 NaN 隱藏）
%       .branch_x(1:3)      對應的決策點 X 標記
%       .branch_label(1:3)  X 標記旁的文字，標示「到這個決策點的距離」
%       .colors_by_type     type 字串 -> RGB 顏色的對照 struct，供
%                            update_decision_graph_plot.m 查色用
%       .legend_straight / .legend_change_left / .legend_change_right /
%       .legend_decision_x  圖例用的代表線／標記 handle，若呼叫端自己的
%                            legend() 是用明確 handle 清單（例如
%                            STEP4_Animation_MultiView.m），要記得把這幾個
%                            handle 加進清單，圖例才會顯示
% =========================================================================

function h = init_decision_graph_plot(ax)
    h.colors_by_type = struct( ...
        'straight',     [0.85 0.85 0.85], ...
        'change_left',  [0.30 0.75 0.95], ...
        'change_right', [0.95 0.60 0.20]);

    h.dot = plot(ax, NaN, NaN, 'o', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'w', ...
        'MarkerSize', 10, 'LineWidth', 1.2, 'DisplayName', '目前位置（已選定）');

    h.branch_line  = gobjects(1, 3);
    h.branch_x     = gobjects(1, 3);
    h.branch_label = gobjects(1, 3);   % 每個 X 標記旁邊標示「到這個決策點的距離」
    for i = 1:3
        h.branch_line(i) = plot(ax, NaN, NaN, '-', 'LineWidth', 2.2, 'HandleVisibility', 'off');
        h.branch_x(i)    = plot(ax, NaN, NaN, 'x', 'MarkerSize', 14, 'LineWidth', 3, 'HandleVisibility', 'off');
        h.branch_label(i) = text(ax, NaN, NaN, '', 'Color', 'w', 'FontSize', 9, ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    end

    % 圖例只需要各類型畫一條代表線（不隨資料更新，純粹讓 legend 顯示三種顏色的意義）。
    % 這 4 個 handle 額外存進 h.legend_*，因為呼叫端（例如 STEP4_Animation_
    % MultiView.m）自己的 legend() 是用「明確指定 handle 清單」的寫法，不會
    % 自動撿到這裡建立的 DisplayName，需要呼叫端自己把這些 handle 加進清單。
    h.legend_straight     = plot(ax, NaN, NaN, '-', 'Color', h.colors_by_type.straight,     'LineWidth', 2.2, 'DisplayName', '直行');
    h.legend_change_left  = plot(ax, NaN, NaN, '-', 'Color', h.colors_by_type.change_left,  'LineWidth', 2.2, 'DisplayName', '變換到左車道');
    h.legend_change_right = plot(ax, NaN, NaN, '-', 'Color', h.colors_by_type.change_right, 'LineWidth', 2.2, 'DisplayName', '變換到右車道');
    h.legend_decision_x   = plot(ax, NaN, NaN, 'x', 'Color', [1 1 1], 'MarkerSize', 14, 'LineWidth', 3, 'DisplayName', '下一個決策點');
end
