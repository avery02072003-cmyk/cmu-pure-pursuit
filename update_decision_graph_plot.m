% =========================================================================
% 檔案名稱: update_decision_graph_plot.m
%
% 功能：把 init_decision_graph_plot.m 建立的圖形物件，用這一輪的
%       車輛狀態與 generate_decision_branches.m 產生的分支資料更新畫面。
%       分支數量可能是 2 或 3（依車道而定，見 generate_decision_branches.m
%       檔頭的分支規則），沒用到的第 3 個槽位會被設成 NaN 隱藏，不會殘留
%       上一輪的線條。
%
% 輸入：
%   h        : init_decision_graph_plot.m 回傳的 handle struct
%   state    : [x, y, yaw, v] 車輛目前狀態（黑點畫在 state(1:2)）
%   branches : generate_decision_branches.m 回傳的分支 struct array
%              （2 或 3 個元素）
%
% 輸出：
%   h : 原樣傳回（MATLAB 圖形物件 handle 是傳址更新，這裡回傳純粹方便
%       呼叫端維持慣用的 h = update_decision_graph_plot(h, ...) 寫法）
% =========================================================================

function h = update_decision_graph_plot(h, state, branches)
    set(h.dot, 'XData', state(1), 'YData', state(2));

    n = numel(branches);
    for i = 1:3
        if i <= n
            b = branches(i);
            color = h.colors_by_type.(b.type);
            set(h.branch_line(i), 'XData', b.x, 'YData', b.y, 'Color', color);
            set(h.branch_x(i), 'XData', b.decision_point(1), 'YData', b.decision_point(2), 'Color', color);
        else
            % 這個槽位這一輪沒有對應的分支（例如在左右車道時只有 2 條），隱藏起來
            set(h.branch_line(i), 'XData', NaN, 'YData', NaN);
            set(h.branch_x(i), 'XData', NaN, 'YData', NaN);
        end
    end
end
