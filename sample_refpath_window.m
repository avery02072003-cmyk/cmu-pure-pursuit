% =========================================================================
% 檔案名稱: sample_refpath_window.m
%
% 功能：從母路徑（環形、首尾相接）上找一個點的錨定位置，往前擷取一段
%       弧長窗口。這段邏輯原本寫在 generate_local_paths.m 步驟 1-2 裡，
%       這裡抽成獨立、可重用的函式，供 generate_decision_branches.m
%       （離散車道決策分支路徑，新功能）呼叫。
%
% ⚠ 刻意不修改 generate_local_paths.m 去呼叫這個新函式：
%   generate_local_paths.m 這次已經用完整模擬驗證過即時追蹤結果
%   （CTE RMS 0.0069m 等），把它重構成呼叫共用函式雖然邏輯上等價，
%   但對已驗證檔案會引入不必要的回歸風險，所以現階段兩邊各自保留一份
%   邏輯相同的程式碼（generate_local_paths.m 維持原樣，本檔案是給新
%   功能專用的獨立版本）。之後如果要消除這個重複，應該另外開一個
%   改動、單獨驗證，不要跟新功能的改動混在一起。
%
% 演算法（與 generate_local_paths.m 步驟 1-2 完全相同）：
%   1. 對母路徑做全域最近點搜尋，找到 xy 的錨定索引
%   2. 從錨定索引往前走，累積弧長直到達到 horizon_m（環形路線用 mod
%      處理路線接縫）
%   3. 濾掉近乎重複的點（避免下游 interp1 對重複弧長座標出錯），並
%      回傳沿窗口累積的弧長座標 s_win，供 resample_window_by_arclength.m
%      使用
%
% 輸入：
%   xy       : [x, y] 世界座標，窗口的起點錨定位置
%   refpath  : 母路徑 struct，含 .x .y .phi（環形路線，首尾相接）
%   horizon_m: 往前擷取的窗口弧長長度 (m)
%
% 輸出：
%   window_x, window_y, window_phi : 窗口內的母路徑點（已濾除近乎重複點，
%                                     phi 已 unwrap 避免跨 -pi/+pi 問題）
%   s_win                           : 對應每個窗口點的累積弧長座標
%                                     （s_win(1)=0）
% =========================================================================

function [window_x, window_y, window_phi, s_win] = sample_refpath_window(xy, refpath, horizon_m)
    Nref = length(refpath.x);

    % ---- 1. 全域最近點搜尋，找錨定索引 ----
    dist_all = hypot(refpath.x - xy(1), refpath.y - xy(2));
    [~, idx_near] = min(dist_all);

    % ---- 2. 從錨定索引往前走，累積弧長直到涵蓋 horizon_m（環形路線用 mod 處理接縫）----
    idx_list = idx_near;
    arc = 0;
    i_cur = idx_near;
    while arc < horizon_m && numel(idx_list) < Nref
        i_nxt = mod(i_cur, Nref) + 1;
        arc = arc + hypot(refpath.x(i_nxt)-refpath.x(i_cur), refpath.y(i_nxt)-refpath.y(i_cur));
        idx_list(end+1) = i_nxt; %#ok<AGROW>
        i_cur = i_nxt;
    end

    window_x   = refpath.x(idx_list);
    window_y   = refpath.y(idx_list);
    window_phi = unwrap(refpath.phi(idx_list));   % 沿窗口 unwrap，避免跨 -pi/+pi 內插出錯

    % 濾掉近乎重複的點，避免下游 interp1 對重複弧長座標出錯
    d_step = hypot(diff(window_x), diff(window_y));
    keep = [true; d_step > 1e-9];
    window_x   = window_x(keep);
    window_y   = window_y(keep);
    window_phi = window_phi(keep);

    s_win = [0; cumsum(hypot(diff(window_x), diff(window_y)))];
end
