% =========================================================================
% 檔案名稱: estimate_current_lane_id.m
%
% 功能：給定車輛（或任何一點）的世界座標位置，判斷它目前落在「左/中/右」
%       哪一條車道，回傳車道代號 lane_id ∈ {-1, 0, +1}（-1=右車道，
%       0=中間車道，+1=左車道，符號跟 shift_waypoints_lateral.m 的
%       「正值＝左」慣例一致）。
%
%       這個「目前在哪條車道」的概念，在既有的即時局部路徑生成
%       （generate_local_paths.m）架構裡完全不存在——那套架構只有一個
%       扁平的 N 條候選路徑陣列跟一個 active_path_idx 索引，重新生成時
%       索引意義就重置了，沒有「車道身分」這件事。本函式是離散決策分支
%       路徑視覺化（generate_decision_branches.m）新增的概念，只有這個
%       新功能會用到，不影響、也不需要整合進現有的模擬主迴圈。
%
% 算法（沿用專案裡已經出現三次的「母路徑最近點投影＋法線方向算橫向
% 偏移」公式，分別是 main_pure_pursuit_sim.m 算 cte、select_best_path.m
% 的 compute_cte、STEP4_Animation_MultiView.m 的 trailer_lat_offset）：
%       1. 對母路徑做最近點搜尋，找到最近點索引 idx
%       2. 取最近點的航向角 yaw_ref = refpath.phi(idx)
%       3. 把「目標點 - 最近點」的位移向量投影到路徑法線方向
%          (-sin(yaw_ref), cos(yaw_ref))，得到橫向偏移 lateral_offset
%          （正值＝在路徑左側，負值＝右側，跟 shift_waypoints_lateral.m
%          的正負號慣例一致）
%       4. lateral_offset 除以 lane_width 四捨五入，得到落在第幾條車道，
%          限制在 {-1, 0, 1}（本專案固定只有 3 條車道，不做更多層）
%
% 輸入：
%   xy      : [x, y] 世界座標
%   refpath : 母路徑 struct，含 .x .y .phi
%   params  : 需要 lane_width
%
% 輸出：
%   lane_id : -1（右車道）、0（中間車道）、或 +1（左車道）
% =========================================================================

function lane_id = estimate_current_lane_id(xy, refpath, params)
    dist2 = (refpath.x - xy(1)).^2 + (refpath.y - xy(2)).^2;
    [~, idx] = min(dist2);

    yaw_ref = refpath.phi(idx);
    dx = xy(1) - refpath.x(idx);
    dy = xy(2) - refpath.y(idx);
    lateral_offset = -sin(yaw_ref)*dx + cos(yaw_ref)*dy;

    lane_id = round(lateral_offset / params.lane_width);
    lane_id = max(-1, min(1, lane_id));
end
