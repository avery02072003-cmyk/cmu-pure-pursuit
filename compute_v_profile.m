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
%       這裡乘上 params.a_lat_margin（<=1 的安全係數，預設 0.85），變成
%           v_lat = sqrt(a_lat_max * a_lat_margin / |kappa|)
%       原因：如果直接用 a_lat_max 當規劃目標，車輛會被規劃成「剛好貼著
%       物理極限走」，v_lat_cmd 本身就等於 a_lat_max，完全沒有餘裕。
%       實際車輛不可能完美跟上規劃曲線（縱向速度只能以 a_dec_max 的
%       斜率逼近目標、Ts 離散取樣本身就有落差），只要車速比規劃值多快
%       一點點，pure_pursuit_controller.m 的側向加速度硬限制（同樣是
%       |v²·kappa|<=a_lat_max）就會被觸發、把轉向角砍掉，車輛會在還沒
%       減到規劃速度前就先轉不動、直線衝出母路徑（這是 v_des 提高到
%       25m/s 後車輛過彎失控的根因，實測驗證：貼著零餘裕的 a_lat_max
%       規劃，車輛進彎當下 v²·kappa 幾乎精確等於 a_lat_max，任何微小
%       落差都會觸發轉向失效）。乘上 0.85 讓規劃速度本身就留一點緩衝，
%       車速即使沒有完美貼著規劃曲線走，也不會一碰彎道就立刻觸發轉向
%       安全網。這是規劃端的「事前預防」，跟 pure_pursuit_controller.m
%       新增的 v_lat_limit（事後即時回饋，見該檔案步驟十說明）是互補
%       關係，不是取代——事前有餘裕可以大幅降低觸發頻率，事後回饋則是
%       萬一真的觸發時，讓車速能立刻反應、不用等到下次 replan。
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
%       ds 由呼叫端傳入，是候選路徑逐點的真實弧長間距（hypot(diff(x),
%       diff(y))），不是假設值。這裡曾經寫死用 ds=0.1 近似，但候選路徑
%       實際點間距（由 stitch_local_path.m 的 Newton-Raphson 曲率擬合
%       密度決定，不是固定值）量測出來中位數只有約 0.033m——用 0.1
%       等於讓 backward pass 誤以為每兩點之間有 3 倍的煞車距離，使規劃
%       出來的減速曲線比實際物理需要的寬鬆約 sqrt(3)≈1.7 倍，車速越高
%       這個誤差的絕對影響越大（這正是 v_des 從 6m/s 提高到 25m/s 後
%       車輛在彎道前煞車不及、直接偏出母路徑的根因之一）。改成接收真實
%       ds，作法呼應 main_pure_pursuit_sim.m 開頭「整條路線」那段
%       backward/forward pass（用 s_arc 累積弧長算出真實 ds）本來就用
%       對的寫法，現在讓即時候選路徑版也用同一套邏輯，兩處不再分岔。
%
% 輸入：
%   kappa  : Nx1 曲率序列（compute_path_curvature.m 算出）
%   params : 需要 a_lat_max, a_lat_margin, v_des, v_profile_min, a_dec_max,
%            a_acc_max（見 STEP1_VehicleParameters.m 第 8、9 節）
%   ds     : (N-1)x1，路徑逐點間的真實弧長間距，ds(i) 對應第 i 點到
%            第 i+1 點的距離（跟 kappa 同一組路徑點、同一個索引順序）。
%            呼叫端在算完 path_x/path_y 之後用 hypot(diff(path_x),
%            diff(path_y)) 算出，兩個呼叫端都已經有這兩個變數在作用域內。
%
% 輸出：
%   v_profile : Nx1，每一點允許的速度上限 (m/s)
%
% 呼叫端：my_multi_path.m、generate_local_paths.m（每條候選路徑都要規劃
%         自己的速度剖面，因為不同候選路徑的曲率、點間距都不同）
% =========================================================================

function v_profile = compute_v_profile(kappa, params, ds)
    N = length(kappa);
    ds = max(ds, 1e-6);   % 防止除以零（重複點的情況），跟 main_pure_pursuit_sim.m 開頭那段同樣的防呆
    v_lat   = sqrt(params.a_lat_max * params.a_lat_margin ./ max(abs(kappa), 1e-4));   % (1) 側向加速度限速（留安全餘裕，見檔頭說明）
    v_hitch = compute_hitch_speed_cap(kappa, params);            % (2) 鉸接角限速
    v_c = min(min(v_lat, v_hitch), params.v_des);
    v_c = max(v_c, params.v_profile_min);
    v_profile = v_c;

    % (3a) backward pass：由終點往起點掃描，確保進入急彎前已經減速到位
    % ds(i) 是第 i 點到第 i+1 點的真實距離，即這一段「留給煞車用」的弧長
    for i = N-1:-1:1
        v_allow = sqrt(v_profile(i+1)^2 + 2*params.a_dec_max*ds(i));
        v_profile(i) = min(v_profile(i), v_allow);
    end
    % (3b) forward pass：由起點往終點掃描，限制加速率不超過物理上限
    % ds(i-1) 是第 i-1 點到第 i 點的真實距離
    for i = 2:N
        v_allow = sqrt(v_profile(i-1)^2 + 2*params.a_acc_max*ds(i-1));
        v_profile(i) = min(v_profile(i), v_allow);
    end
end
