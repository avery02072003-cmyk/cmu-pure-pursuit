% =========================================================================
% 檔案名稱: lane_change_offset_profile.m
%
% 功能：算出「換道曲線」沿弧長 s 的側向偏移量 d(s)，以及對應的航向修正量
%       dphi_correction(s)。這是 generate_decision_branches.m 產生換道分支
%       的數學核心，純數學運算，不涉及任何座標幾何轉換（幾何轉換交給
%       shift_waypoints_lateral_profile.m 做）。
%
% 偏移曲線形狀（平滑階梯函數，S 型曲線）：
%       t = clamp(s / ramp_distance, 0, 1)
%       f(t) = 3*t^2 - 2*t^3            （Hermite smoothstep，f(0)=0, f(1)=1，
%                                          兩端斜率 f'(0)=f'(1)=0）
%       d(s) = offset_start + (offset_end - offset_start) * f(t)
%
%   用平滑階梯而不是直線斜坡，是因為兩端斜率為 0 的特性：換道曲線一開始
%   側向速度是 0（車輛還沒開始橫移），換道結束時側向速度也是 0（車輛已經
%   對齊新車道、不再橫移），這才像真實駕駛的換道動作；直線斜坡則會在
%   換道開始/結束的瞬間讓側向偏移速度突然從 0 跳到非 0（或反過來），
%   對應到車輛動態上會是不合理的側向加速度尖峰。
%
% 為什麼需要 dphi_correction（這是本函式存在的主要原因）：
%   shift_waypoints_lateral.m／stitch_local_path.m 現有的做法是「側向平移
%   一個常數距離，但沿用平移前中心線的航向角」，這個近似只在偏移量是
%   常數、遠小於曲率半徑時成立。但這裡的偏移量 d(s) 是隨弧長變化的斜坡，
%   偏移的變化率本身就會產生真實的切線方向偏轉：
%
%       設中心線在 s 處的切線方向為 T(s)（沿弧長參數化的單位切向量），
%       法線方向為 n(s)（左手側）。偏移後的曲線 P(s) = P_center(s) + d(s)*n(s)，
%       對 s 微分（用 Frenet-Serret 公式 dn/ds = -kappa(s)*T(s)）：
%
%           dP/ds = (1 - d(s)*kappa(s))*T(s) + d'(s)*n(s)
%
%       在局部曲率 kappa(s) 很小（本專案賽道已放大 5 倍，彎道都很平緩）
%       的情況下，(1 - d*kappa) ≈ 1，於是偏移曲線的切線方向相對中心線
%       切線方向的偏轉角 Δθ 滿足：
%
%           tan(Δθ) ≈ d'(s) / 1 = d'(s)   →   Δθ ≈ atan(d'(s))
%
%   這就是 dphi_correction 的由來。用實際數字驗證量級：lane_width=3.5m、
%   lane_change_time_s=4s、v_des=6m/s 時，換道距離 ramp_distance=24m，
%   偏移總變化量 3.5m，f'(t) 在 t=0.5 時取最大值 1.5（f'(t)=6t-6t^2，
%   峰值在 t=0.5 處 = 1.5），對應 d'(s) 峰值 = 3.5*1.5/24 ≈ 0.219，
%   Δθ_peak = atan(0.219) ≈ 12.5°——這個量級如果不修正，會讓
%   stitch_local_path.m 每個換道 segment 都要硬吃掉這個角度落差，
%   擬合出的曲線在 segment 交界處會看起來有稜有角。加上這個修正後，
%   stitch_local_path.m 拿到的目標航向就已經是「真正沿著偏移曲線走」
%   該有的切線方向，不需要額外硬轉。
%
%   f(t) 在 t=0 與 t=1 處斜率剛好是 0，所以 d'(s) 在 ramp_distance 範圍
%   之外自然等於 0（t 被 clamp 在邊界，f'(t)=6t-6t^2 在 t=0,1 都算出 0），
%   不需要另外用 if/else 特別處理範圍外的情況。
%
% 輸入：
%   s             : Nx1 或 1xN 弧長座標（從換道分支起點算起，s=0 為起點）
%   ramp_distance : 換道斜坡的總距離（通常是 v*lane_change_time_s，
%                   已在呼叫端限制在合理範圍）
%   offset_start  : 起點側向偏移量（通常是目前車道的偏移，例如 0 或 ±lane_width）
%   offset_end    : 終點側向偏移量（目標車道的偏移）
%
% 輸出：
%   d_profile       : 對應每個 s 的側向偏移量
%   dphi_correction : 對應每個 s 的航向修正量（rad），加進中心線航向角
%                     後才是換道曲線真正的目標航向
% =========================================================================

function [d_profile, dphi_correction] = lane_change_offset_profile(s, ramp_distance, offset_start, offset_end)
    ramp_distance = max(ramp_distance, 1e-6);   % 防止除以 0

    t = min(max(s(:) / ramp_distance, 0), 1);
    f = 3*t.^2 - 2*t.^3;
    fprime = 6*t - 6*t.^2;

    d_profile = offset_start + (offset_end - offset_start) * f;

    dprime_ds = (offset_end - offset_start) * fprime / ramp_distance;   % d(d)/ds，鏈式法則
    dphi_correction = atan(dprime_ds);

    % 還原成輸入 s 的形狀（列向量或行向量皆可）
    d_profile = reshape(d_profile, size(s));
    dphi_correction = reshape(dphi_correction, size(s));
end
