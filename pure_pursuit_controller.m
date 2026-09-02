% =========================================================================
% pure_pursuit_controller.m
% Pure Pursuit 追蹤控制器
%
% 功能說明：
%   給定當前車輛狀態，從參考路徑上找到 look-ahead 目標點，
%   計算 pure pursuit 幾何轉向角，加上航向誤差回授修正，
%   最後做側向加速度硬限制，輸出最終前輪轉向角指令。
%
% 輸入：
%   x, y   : 車輛當前位置 (m)
%   yaw    : 車輛當前航向角 (rad)
%   v      : 車輛當前速度 (m/s)
%   refpath: 參考路徑結構，包含 .x .y .phi .kappa .v_profile
%   params : 控制器參數結構
%   idx_prev: 上一步的最近點索引（用於限制搜尋範圍）
%
% 輸出：
%   delta       : 前輪轉向角指令 (rad)
%   idx_target  : look-ahead 目標點在 refpath 的索引
%   idx_near    : 最近點在 refpath 的索引
%   Ld          : 本步動態前視距離 (m)
%   alpha       : 車輛到目標點的方位角誤差 (rad)
%   v_lat_limit : 這一步的轉向幾何（未被側向加速度限制前）需要的曲率，
%                 換算出來「車輛最快只能開多快」的速度上限 (m/s)。供
%                 main_pure_pursuit_sim.m 的縱向速度控制即時取用（見
%                 步驟十的完整推導）——不是拿來取代 compute_v_profile.m
%                 事先規劃好的 v_profile，而是跟它取更保守的那個當這
%                 一步的即時目標，讓速度控制不用等到下次 replan 才
%                 發現「這一步其實已經轉不動了」。
%
% 控制器架構：
%   delta = delta_pp + delta_fb
%   其中：
%     delta_pp = atan2(2*L*sin(alpha), Ld)      ← Pure Pursuit 幾何
%     delta_fb = -Kh * heading_error / v        ← 航向誤差回授
%
% 對應論文元素：
%   - 自適應前視距離：Ld = Ld0 + kv*v - kappa_gain*kappa
%   - 側向加速度硬限制：|v²·kappa_cmd| ≤ a_lat_max
%
% delta_pp = atan2(2*L*sin(alpha), Ld) 的幾何推導：
%   Pure Pursuit 的基本假設是「車輛沿著一個圓弧行駛，剛好通過前方的
%   look-ahead 目標點」。設這個圓弧半徑為 R，圓心在車輛後軸的垂直
%   方向上（bicycle model 的轉向幾何：後軸走圓弧、前輪轉角 delta 滿足
%   R = L/tan(delta)，L 為軸距）。
%
%   車輛後軸位置、目標點、圓心三者構成一個等腰三角形（兩股都是半徑
%   R），目標點到車輛的直線距離就是前視距離 Ld，alpha 是這條直線
%   相對車輛航向的夾角。用「弦長 - 圓心角」關係（正弦定理）：
%       Ld = 2*R*sin(alpha)          →  R = Ld / (2*sin(alpha))
%   （這個關係可以用「圓周角是圓心角一半」的幾何性質，或直接對等腰
%   三角形做垂直平分線推導得到，是 Pure Pursuit 演算法最核心的一步。）
%
%   有了 R，代入 bicycle model 的轉向幾何 R = L/tan(delta)：
%       tan(delta) = L/R = L * 2*sin(alpha) / Ld = 2*L*sin(alpha) / Ld
%       delta = atan2(2*L*sin(alpha), Ld)
%   用 atan2 而不是 atan，是為了正確處理 alpha 為負值（目標點在車輛
%   右側）時 delta 的正負號與角度範圍。
% =========================================================================
function [delta, idx_target, idx_near, Ld, alpha, v_lat_limit] = pure_pursuit_controller(x, y, yaw, v, refpath, params, idx_prev)

% 路徑總點數
N = length(refpath.x);

% -------------------------------------------------------------------------
% 步驟一：計算車輛到路徑所有點的歐氏距離
% hypot(a,b) = sqrt(a²+b²)，向量化計算全路徑距離
% -------------------------------------------------------------------------
dist_all = hypot(refpath.x - x, refpath.y - y);

% -------------------------------------------------------------------------
% 步驟二：在搜尋視窗內找最近點 idx_near
% 搜尋視窗：從 idx_prev 往前看 80 點（避免全域搜尋，加速計算）
% 使用 mod 處理路徑端點的環形循環
% -------------------------------------------------------------------------
search_len = 80;    % 搜尋視窗長度（點數）
% 建立搜尋索引範圍（環形）
search_range = mod((idx_prev-1):(idx_prev-1+search_len), N) + 1;
% 在視窗內找距離最小的點
[~, local_min] = min(dist_all(search_range));
idx_near = search_range(local_min);  % 最近點全域索引

% -------------------------------------------------------------------------
% 步驟三：取得最近點的路徑曲率
% 用於計算自適應前視距離 Ld（曲率大 → Ld 縮短，應對急彎）
% -------------------------------------------------------------------------
kappa_near = 0;     % 預設曲率為 0（直路）
if isfield(refpath, 'kappa')
    kappa_near = abs(refpath.kappa(idx_near));  % 取絕對值（只關心曲率大小）
end

% -------------------------------------------------------------------------
% 步驟四：計算自適應前視距離 Ld
% 公式：Ld = Ld0 + kv*v - kappa_gain*kappa
%   Ld0        : 基礎前視距離，保證最小看前距離
%   kv*v       : 速度補償，速度越快看越遠（反應時間補償）
%   kappa_gain*kappa : 曲率縮短，彎道時縮短 Ld 避免切彎
% 最後截斷到 [Ld_min, Ld_max_dyn] 安全範圍
%
% Ld_max_dyn = max(Ld_max, Ld_time_cap_s * v)：上限不再是固定值。
% params.Ld_max 本身當作「低速時的下限保障」，永遠不會比這個值更低；
% Ld_time_cap_s*v 則讓上限隨車速線性成長。原因：固定上限（例如 8m）
% 在設計速度（v_des=6m/s）時不會被頂到（Ld_raw=Ld0+kv*6=3.8m < 8m），
% 但車速一旦大幅提高（例如 25m/s），Ld_raw=Ld0+kv*25=9.5m 就會被硬砍
% 到 8m，前視「時間」（Ld/v）從設計時的 0.63s 腰斬到只剩 0.32s——
% pure pursuit 在前視時間過短時會轉向過度激進、容易在彎道切出去，這正是
% v_des 提高到 25m/s 後車輛過彎失控的根因之一。用 Ld_time_cap_s（而不是
% 直接調高固定上限）是因為它讓上限本身跟著車速走，不會在某個更高的速度
% 又重新頂到新的固定天花板；且在 v=6m/s 時 max(8, 0.4*6=2.4)=8，跟修改
% 前完全一樣（v_des=6 是已驗證過的設計基準點，零行為變化）。
% -------------------------------------------------------------------------
Ld = params.Ld0 + params.kv * abs(v) - params.kappa_gain * kappa_near;
Ld_max_dyn = max(params.Ld_max, params.Ld_time_cap_s * abs(v));
Ld = max(params.Ld_min, min(Ld, Ld_max_dyn));  % 截斷到安全範圍

% -------------------------------------------------------------------------
% 步驟五：用弧長累加法搜尋 look-ahead 目標點 idx_target
% 原理：從 idx_near 出發沿路徑前進，累加各段弧長，
%       找第一個累加弧長 ≥ Ld 的點作為目標點
% 優點：確保目標點是沿路徑前進方向的點，不會因為弧形路徑
%       而找到幾何距離近但路徑上在後面的點
% -------------------------------------------------------------------------
arc = 0;              % 累計弧長初始化
idx_target = idx_near; % 預設目標點為最近點（若路徑太短無法達到 Ld）
for step_i = 1:N-1
    % 計算第 step_i 段的起點和終點索引（環形）
    i_cur = mod(idx_near - 1 + step_i - 1, N) + 1;  % 本段起點
    i_nxt = mod(idx_near - 1 + step_i,     N) + 1;  % 本段終點
    % 累加本段弧長
    arc = arc + hypot(refpath.x(i_nxt) - refpath.x(i_cur), ...
                      refpath.y(i_nxt) - refpath.y(i_cur));
    % 一旦累計弧長達到 Ld，取當前終點為目標點
    if arc >= Ld
        idx_target = i_nxt;
        break;
    end
end

% 目標點的世界座標
tx = refpath.x(idx_target);  % 目標點 x (m)
ty = refpath.y(idx_target);  % 目標點 y (m)

% -------------------------------------------------------------------------
% 步驟六：計算 Pure Pursuit 幾何轉向角 delta_pp
% alpha：車輛座標系中，目標點相對車輛的方位角
%        = atan2(ty-y, tx-x) - yaw
% delta_pp 公式推導（Bicycle Model 幾何）：
%        delta_pp = atan2(2*L*sin(alpha), Ld)
% 物理意義：讓車輛在前視距離 Ld 內轉向到達目標點所需的轉向角
% -------------------------------------------------------------------------
alpha = atan2(ty - y, tx - x) - yaw;           % 目標點方位角誤差
alpha = atan2(sin(alpha), cos(alpha));          % 正規化到 (-π, π)
delta_pp = atan2(2 * params.L1 * sin(alpha), Ld); % Pure Pursuit 幾何公式

% -------------------------------------------------------------------------
% 步驟七：估計目標點的局部切線方向（用於航向回授）
% 方法：用目標點前後各 hw 個點的座標差估計切線方向
%       比直接使用 refpath.phi 更穩定（避免 phi 採樣雜訊）
% -------------------------------------------------------------------------
hw = 2;   % 半視窗大小（前後各取 2 點）
i_a = mod(idx_target - 1 - hw, N) + 1;  % 目標點前 hw 個點的索引
i_b = mod(idx_target - 1 + hw, N) + 1;  % 目標點後 hw 個點的索引
% 用兩點座標差的 atan2 估計切線方向
yaw_target = atan2(refpath.y(i_b) - refpath.y(i_a), ...
                   refpath.x(i_b) - refpath.x(i_a));

% -------------------------------------------------------------------------
% 步驟八：計算航向誤差回授修正量 delta_fb
% 目的：修正車頭方向與目標點切線方向的夾角
%       讓 delta_pp（位置修正）和 delta_fb（方向修正）對齊同一個參考點
% 公式：delta_fb = -Kh * he_now / v
%   he_now : 車輛航向角與目標點切線方向的夾角
%   Kh     : 航向回授增益（越大修正越積極）
%   /v     : 速度歸一化（速度越快同樣誤差只需較小轉向）
% -------------------------------------------------------------------------
% 航向誤差：車輛偏航角相對目標點切線方向
he_now = atan2(sin(yaw - yaw_target), cos(yaw - yaw_target));
v_safe = max(v, 1.0);   % 防止低速時除以過小的速度值導致修正爆炸
delta_fb = -params.Kh * he_now / v_safe;  % 航向回授修正量

% -------------------------------------------------------------------------
% 步驟九：合成最終轉向角指令
% delta = delta_pp（位置修正）+ delta_fb（方向修正）
% -------------------------------------------------------------------------
delta = delta_pp + delta_fb;

% 物理限制：前輪最大轉向角 ±35°（轉換為弧度）
delta_max_phys = deg2rad(35);
delta = max(-delta_max_phys, min(delta, delta_max_phys));

% -------------------------------------------------------------------------
% 步驟十：側向加速度硬限制（論文約束）
% 即使 delta 在物理範圍內，也要確保 |v²·kappa| ≤ a_lat_max
% 若超出，反推出允許的最大 kappa，重新計算 delta
%
% v_lat_limit（新增輸出，供呼叫端即時修正車速用）：
%   這道限制是 |v²·kappa_cmd| <= a_lat_max，換句話說，在「目前這個
%   轉向幾何真正需要的曲率 kappa_cmd」之下，車輛最快只能開到
%   v_lat_limit = sqrt(a_lat_max / |kappa_cmd|)，超過這個速度，轉向就會
%   被這道安全網砍掉、車輛實際上轉不出目標曲率所需要的彎。
%
%   這個限制本身沒有錯（物理上車輛真的做不到更急的轉彎），問題出在：
%   compute_v_profile.m 事先規劃好的 v_profile，只有在每次 T_replan
%   （0.5秒）才重新規劃一次，如果車輛「目前實際速度」因為任何原因
%   （減速指令本身也有 a_dec_max 斜率限制、規劃時的近似誤差等）還沒
%   跟上這條曲線該有的速度，車輛會在還沒減到 v_lat_limit 之前就先
%   撞上這道轉向安全網——這時候只是被動地砍轉向角、繼續照原速度直行，
%   要等到「原本規劃的減速斜率」慢慢把車速降下來，這段時間車輛完全
%   沒辦法轉彎，會直線衝出母路徑（這是 v_des 提高到 25m/s 後車輛過彎
%   失控、且失控後修正過度變成來回振盪的根因——實測驗證見
%   main_pure_pursuit_sim.m 呼叫端的完整推導與數據）。
%
%   修正方式：把 v_lat_limit 傳回呼叫端，讓縱向速度跟隨控制器在每一步
%   都拿它跟預先規劃的 v_ref_now 取更保守（較小）的那個當目標，這樣
%   車速的目標值會「即時」反映「這一步的轉向到底還撞不撞得到這道安全
%   網」，不用等到下一次 replan 才更新；車速本身仍然只能以 a_dec_max
%   的物理減速率逼近這個目標（沒有違反任何運動學限制，只是讓目標值
%   更即時、更準確，不再依賴半秒才更新一次、可能偏樂觀的預先規劃值）。
% -------------------------------------------------------------------------
kappa_cmd = tan(delta) / params.L1;    % 由轉向角計算曲率指令
a_lat_cmd  = v^2 * kappa_cmd;         % 估算側向加速度
v_lat_limit = sqrt(params.a_lat_max / max(abs(kappa_cmd), 1e-4));  % 這個轉向幾何允許的最高速度

if abs(a_lat_cmd) > params.a_lat_max
    % 超出限制：反推最大允許曲率，再換算回轉向角
    kappa_limited = sign(a_lat_cmd) * params.a_lat_max / max(v^2, 1e-3);
    delta = atan(kappa_limited * params.L1);  % 換算回轉向角
end

% 再次截斷確保物理限制（防止 atan 結果超出 ±35°）
delta = max(-delta_max_phys, min(delta, delta_max_phys));

end
