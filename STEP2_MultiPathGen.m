% =========================================================================
% 檔案名稱: STEP2_MultiPathGen.m
%
% 功能：獨立的「整條路線」多候選路徑預覽腳本，非必要執行步驟（不影響
%       main_pure_pursuit_sim.m 的模擬結果），純粹用來視覺化確認
%       my_multi_path.m 產生的 N 條側向平移候選路徑長什麼樣子、是否平滑、
%       側向間距是否符合預期。這在調參（例如改 lane_width、N_paths、
%       local_wp_spacing 等）之後，是最快能「肉眼確認候選路徑生成邏輯
%       有沒有壞掉」的檢查方式。
%
% 使用方式：
%   須先執行過 STEP1_VehicleParameters.m（產生 vehicle_params.mat）與
%   已存在 reference_path.mat（母路徑），本腳本才能執行。
%
% 跟 main_pure_pursuit_sim.m 的關係：
%   本腳本呼叫 my_multi_path.m（整條路線版候選路徑生成），跟
%   main_pure_pursuit_sim.m 主迴圈裡呼叫的 generate_local_paths.m
%   （即時局部窗口版）是兩個不同的呼叫端，但底層共用同一套
%   shift_waypoints_lateral / stitch_local_path / compute_path_curvature /
%   compute_v_profile 核心模組（見 my_multi_path.m 檔頭說明），所以
%   本腳本畫出來的候選路徑平滑度，可以直接反映主模擬裡即時生成的
%   候選路徑品質。
% =========================================================================

clear; clc; close all;
load('vehicle_params.mat', 'params');
load('reference_path.mat', 'refpath');

% 把母路徑抽稀成 GPS waypoints（每 50 個密集取樣點取 1 個），降低
% my_multi_path.m 逐段呼叫 my_path() 的次數，同時保留足夠的路徑細節
stride = 50;
gps_wp = [refpath.x(1:stride:end), refpath.y(1:stride:end)];

% 產生 N_paths 條側向平移候選路徑（整條路線，from 起點 to 終點）
path_candidates = my_multi_path(gps_wp, params.N_paths, params);

% ---- 繪圖：把每條候選路徑用不同顏色畫出來，並標示各自的側向偏移量 ----
% 偏移量公式必須跟 my_multi_path.m 實際產生候選路徑用的公式完全一致，
% 才能算出正確的圖例標示數值。my_multi_path.m 的側向偏移範圍涵蓋本車道
% + 左右各 n_side_lanes 條鄰車道（詳見該檔案內「span_half」的計算），
% 不是單純 ±lane_width/2，這裡要用同一套公式反推，不能各自維護一份。
n_side_lanes = 0;
if isfield(params, 'n_side_lanes'), n_side_lanes = params.n_side_lanes; end
span_half = params.lane_width/2 + n_side_lanes * params.lane_width;
offsets = linspace(-span_half, span_half, params.N_paths);

figure; hold on; axis equal; grid on;
colors = lines(params.N_paths);
for i = 1:params.N_paths
    p = path_candidates{i};
    if ~isempty(p) && isfield(p, 'x')
        plot(p.x, p.y, '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
             'DisplayName', sprintf('Path %d (offset=%.2fm)', i, offsets(i)));
    end
end
plot(gps_wp(:,1), gps_wp(:,2), 'k+', 'MarkerSize', 10, 'DisplayName', 'GPS Waypoints');
legend; title('Multi-Path Candidates Preview');
xlabel('X (m)'); ylabel('Y (m)');
