% =========================================================================
% 檔案名稱: compute_v_profile.m
%
% 功能：由一條候選路徑的曲率序列 kappa，規劃出每一點允許的最高速度
%       v_profile，同時滿足三個限制：(1) 側向加速度上限、(2) 鉸接角
%       安全上限、(3) 縱向加速度/減速度上限。這是「先規劃好整條路徑
%       該開多快，再讓車輛跟隨」的速度規劃器，跟橫向的路徑幾何完全
%       分開處理（橫向路徑由 stitch_local_path.m 決定，本函式只管
%       「同一條路徑上、每一點該開多快」）。
%
% 三層限制的推導：
%
%   (1) 側向加速度限制 v_lat（車輛過彎不側滑翻覆的基本物理限制）：
%       圓周運動側向加速度 a_lat = v^2 * kappa，要求 |a_lat| <= a_lat_max，
%       解出速度上限：
%           v_lat = sqrt(a_lat_max / |kappa|)
%
%   (2) 鉸接角安全限制 v_hitch（聯結車特有，一般單車模型沒有這一項）：
%       由 compute_hitch_speed_cap.m 計算，原理是「曲率越大，穩態鉸接角
%       越大（phi_ss ≈ atan(L2*kappa)），越靠近折疊角上限 phi_max 就必須
%       提早降速」，避免車輛還沒轉進彎道、就已經因為曲率過大而必然導致
%       貨櫃甩尾過度。詳細公式推導見 compute_hitch_speed_cap.m 檔頭。
%
%       取 v_lat 與 v_hitch 兩者較小值，並且不超過期望巡航速度 v_des、
%       不低於最低速度 v_profile_min（避免除以趨近 0 的速度導致其他
%       公式數值不穩定）：
%           v_c = min(v_lat, v_hitch, v_des)，並截斷到 >= v_profile_min
%
%   (3) 縱向加速度/減速度限制（forward-backward pass，防止「規劃出的
%       速度曲線」本身在物理上就做不到）：
%       單獨用 (1)(2) 算出來的 v_c 逐點限速，可能會出現「這一點限速
%       6m/s，下一點突然限速 2m/s」這種瞬間大幅降速的曲線，但車輛
%       不可能瞬間減速，所以還要做兩次掃描修正：
%
%       backward pass（由終點往起點掃描，確保「來得及在進入急彎前減速」）：
%           v_profile(i) = min(v_profile(i), sqrt(v_profile(i+1)^2 + 2*a_dec_max*ds))
%       這個公式是等減速度運動學公式 v_i^2 = v_{i+1}^2 + 2*a_dec_max*ds
%       反過來想：如果要在到達 i+1 點時速度不超過 v_profile(i+1)，那麼
%       在 i 點的速度最多只能是這個值（否則從 i 到 i+1 這段距離內、
%       就算用最大減速度 a_dec_max 全力煞車，也來不及降到 v_profile(i+1)）。
%
%       forward pass（由起點往終點掃描，限制「加速率不能超過物理上限」）：
%           v_profile(i) = min(v_profile(i), sqrt(v_profile(i-1)^2 + 2*a_acc_max*ds))
%       同樣是等加速度運動學公式，限制從 i-1 到 i 這段距離內、車輛最多
%       只能用 a_acc_max 加速到多快。
%
%       這裡固定用 ds=0.1（假設等距，簡化計算；候選路徑本身取樣間距
%       跟 0.1m 不完全相等，但因為只是為了让加減速率合理，這個近似
%       對結果影響很小）。
%
% 輸入：
%   kappa  : Nx1 曲率序列（compute_path_curvature.m 算出）
%   params : 需要 a_lat_max, v_des, v_profile_min, a_dec_max, a_acc_max
%            （見 STEP1_VehicleParameters.m 第 8、9 節）
%
% 輸出：
%   v_profile : Nx1，每一點允許的速度上限 (m/s)
%
% 呼叫端：my_multi_path.m、generate_local_paths.m（每條候選路徑都要規劃
%         自己的速度剖面，因為不同候選路徑的曲率不同）
% =========================================================================

function v_profile = compute_v_profile(kappa, params)
    N = length(kappa);
    v_lat   = sqrt(params.a_lat_max ./ max(abs(kappa), 1e-4));   % (1) 側向加速度限速
    v_hitch = compute_hitch_speed_cap(kappa, params);            % (2) 鉸接角限速
    v_c = min(min(v_lat, v_hitch), params.v_des);
    v_c = max(v_c, params.v_profile_min);
    v_profile = v_c;

    % (3a) backward pass：由終點往起點掃描，確保進入急彎前已經減速到位
    for i = N-1:-1:1
        v_allow = sqrt(v_profile(i+1)^2 + 2*params.a_dec_max*0.1);
        v_profile(i) = min(v_profile(i), v_allow);
    end
    % (3b) forward pass：由起點往終點掃描，限制加速率不超過物理上限
    for i = 2:N
        v_allow = sqrt(v_profile(i-1)^2 + 2*params.a_acc_max*0.1);
        v_profile(i) = min(v_profile(i), v_allow);
    end
end
