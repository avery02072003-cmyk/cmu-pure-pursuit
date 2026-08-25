% =========================================================================
% 檔案名稱: resample_window_by_arclength.m
%
% 功能：把 sample_refpath_window.m 取出的窗口點，依指定的弧長座標
%       s_targets 內插抽稀（或加密）。這段邏輯原本寫在 generate_local_paths.m
%       步驟 3 裡（固定用 local_wp_spacing 等間距抽稀），這裡抽成獨立、
%       可重用的函式，並把「要在哪些弧長位置取樣」開放成參數，因為
%       generate_decision_branches.m（新功能）的換道分支需要「換道斜坡
%       區段密、其餘區段疏」的不等間距取樣，不是單一固定間距能表示的。
%
% ⚠ 同樣刻意不修改 generate_local_paths.m 去呼叫這個新函式，理由跟
%   sample_refpath_window.m 檔頭說明一致：避免對已驗證檔案引入回歸風險。
%
% 航向角用內插而非重新估計：直接對 window_phi（母路徑既有、平滑的
% refpath.phi）做內插，而不是用抽稀後的稀疏點重新算 gradient()，避免
% 粗抽稀造成航向估計雜訊——這個做法跟 generate_local_paths.m 步驟 3
% 的理由完全相同。
%
% 輸入：
%   window_x, window_y, window_phi : sample_refpath_window.m 的輸出
%   s_win                           : 對應每個窗口點的累積弧長座標
%   s_targets                       : 要內插取樣的弧長座標（呼叫端自行
%                                      決定間距，可以是等間距，也可以是
%                                      換道分支那種「密疏交錯」的不等間距）
%
% 輸出：
%   x_out, y_out, phi_out : 內插後的路徑點座標與航向角
% =========================================================================

function [x_out, y_out, phi_out] = resample_window_by_arclength(window_x, window_y, window_phi, s_win, s_targets)
    x_out   = interp1(s_win, window_x,   s_targets, 'linear');
    y_out   = interp1(s_win, window_y,   s_targets, 'linear');
    phi_out = interp1(s_win, window_phi, s_targets, 'linear');
end
