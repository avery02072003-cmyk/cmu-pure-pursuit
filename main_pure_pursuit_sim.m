% =========================================================================
% main_pure_pursuit_sim.m
% Pure Pursuit 追蹤模擬主程式
%
% 功能說明：
%   1. 載入參考路徑 (reference_path.mat)
%   2. 估算路徑曲率並進行平滑化
%   3. 依側向加速度限制計算曲率限速
%   4. 執行 forward-backward speed pass（縱向加減速約束）
%   5. 執行 Bicycle Model 閉迴路追蹤模擬
%   6. 輸出追蹤誤差指標並繪圖
%
% 對應論文元素：
%   - 側向加速度約束：v_curve = sqrt(a_lat_max / kappa)
%   - 縱向加速度約束：forward-backward speed pass
%   - 追蹤控制器：pure_pursuit_controller.m
%
% =========================================================================

clear; clc; close all;

% -------------------------------------------------------------------------
% 載入參考路徑資料
% refpath 結構包含：
%   .x   : 路徑 x 座標陣列 (m)
%   .y   : 路徑 y 座標陣列 (m)
%   .phi : 路徑各點的航向角（偏航角）陣列 (rad)
% -------------------------------------------------------------------------
load('reference_path.mat', 'refpath');

% 直接載入已存好的參數，不要呼叫 STEP1（它有 clear 會清掉 refpath）
load('vehicle_params.mat', 'params');

if ~isfield(params,'L1')
    error('vehicle_params.mat 沒有 params.L1');
end


% =========================================================================
% 控制器與車輛參數設定
% =========================================================================

params.Ts = 0.05;       % 模擬時間步長 (s)，對應 20 Hz 控制頻率
params.L  = params.L1;

% --- Pure Pursuit 前視距離參數 ---
% Ld 公式：Ld = Ld0 + kv*v - kappa_gain*kappa
% 速度越快 → Ld 越大（看得越遠）
% 曲率越大 → Ld 越小（看得越近，應對急彎）
params.Ld0 = 2.0;       % 基礎前視距離 (m)
params.kv = 0.3;        % 速度前視補償係數 (m·s/m = s)
params.Ld_min = 1.2;    % 前視距離下限 (m)，防止過近看目標
params.Ld_max = 8.0;    % 前視距離上限 (m)，防止過遠跳過彎道
params.kappa_gain = 6.0;% 曲率前視縮短係數，彎道時縮短 Ld

% --- 航向回授增益 ---
params.Kh = 0.4;        % 航向誤差回授增益，對應 delta_fb = -Kh * he / v
                        % 值越大修正越積極，但可能導致 CTE 惡化

% --- 動態速度規劃參數（對應論文縱向約束）---
params.v_des = 6.0;     % 期望巡航速度 (m/s)
params.v_min = 1.0;     % 最低速度下限 (m/s)，防止除以零
params.a_lat_max = 2.0; % 側向加速度限制 (m/s²)，論文約束 |a_lat| ≤ a_lat_max
params.a_acc_max = 1.0; % 縱向加速度上限 (m/s²)，論文約束 a ≤ a_acc_max
params.a_dec_max = 1.5; % 縱向減速度上限 (m/s²)，論文約束 |a_brk| ≤ a_dec_max

% =========================================================================
% 步驟一：從參考路徑估算曲率 kappa
% 原理：kappa = d(phi)/ds，即航向角對弧長的導數
% =========================================================================

% 計算路徑各點的 x, y 方向差分（用於估算弧長微元 ds）
dx_ref = gradient(refpath.x);  % x 方向的數值梯度
dy_ref = gradient(refpath.y);  % y 方向的數值梯度

% 計算弧長微元 ds = sqrt(dx² + dy²)
ds_ref = hypot(dx_ref, dy_ref);
ds_ref(ds_ref < 1e-6) = 1e-6;  % 防止除以零（對應路徑上重複點）

% 對航向角做 unwrap，消除 -π/+π 跳變，確保梯度計算正確
phi_unwrap = unwrap(refpath.phi);

% 數值計算曲率：kappa = d(phi)/ds
kappa_raw = gradient(phi_unwrap) ./ ds_ref;

% 物理可行性截斷：最大曲率對應最大轉向角 35°
% kappa_max = tan(35°) / L
kappa_max_physical = tan(deg2rad(35)) / params.L1;
kappa_raw(abs(kappa_raw) > kappa_max_physical) = ...
    sign(kappa_raw(abs(kappa_raw) > kappa_max_physical)) * kappa_max_physical;

% 移動平均平滑化曲率（窗寬 9 點），消除數值微分的高頻雜訊
win = 9;
kernel = ones(1, win) / win;    % 等權移動平均濾波器
% 對兩端做邊界填充（用首尾值複製），避免 conv 邊界效應
kappa_smooth = conv([kappa_raw(1)*ones(1,(win-1)/2), kappa_raw(:)', kappa_raw(end)*ones(1,(win-1)/2)], kernel, 'valid');

% 將平滑後的曲率存回 refpath 結構，供 controller 使用
refpath.kappa = kappa_smooth(:);

% =========================================================================
% 步驟二：曲率限速（論文側向加速度約束）
% 公式：v_curve = sqrt(a_lat_max / |kappa|)
% 物理意義：在曲率 kappa 的彎道上，要滿足 |v²·kappa| ≤ a_lat_max
%          最大允許速度就是 sqrt(a_lat_max / |kappa|)
% =========================================================================
v_curve = sqrt(params.a_lat_max ./ max(abs(refpath.kappa), 1e-4));
v_curve = min(v_curve, params.v_des);   % 不超過期望巡航速度
v_curve = max(v_curve, params.v_min);   % 不低於最低速度

% =========================================================================
% 步驟三：計算路徑弧長累積值 s_arc
% 用於 forward-backward speed pass 的距離計算
% =========================================================================
N = length(refpath.x);      % 路徑總點數
s_arc = zeros(N,1);         % 各點到起點的弧長累積值 (m)
for i = 2:N
    % 弧長累加：s(i) = s(i-1) + 兩點間距離
    s_arc(i) = s_arc(i-1) + hypot(refpath.x(i)-refpath.x(i-1), refpath.y(i)-refpath.y(i-1));
end

% =========================================================================
% 步驟四：Backward Pass（減速約束）
% 從路徑終點往起點反推，確保在到達高曲率點之前已充分減速
% 約束：v(i) ≤ sqrt(v(i+1)² + 2·a_dec_max·ds)
% 物理意義：從 i 到 i+1 最多能減速 a_dec_max，所以 i 點的速度不能超過此值
% =========================================================================
v_profile = v_curve(:);     % 初始化速度規劃為曲率限速

for i = N-1:-1:1
    ds = s_arc(i+1) - s_arc(i);    % 本段弧長
    if ds < 1e-6
        continue;   % 跳過重複點
    end
    % 由 i+1 點往回推算 i 點的最大允許速度
    v_allow = sqrt(v_profile(i+1)^2 + 2*params.a_dec_max*ds);
    v_profile(i) = min(v_profile(i), v_allow);  % 取較小值（更嚴格的約束）
end

% =========================================================================
% 步驟五：Forward Pass（加速約束）
% 從路徑起點往終點正推，確保加速不超過 a_acc_max
% 約束：v(i) ≤ sqrt(v(i-1)² + 2·a_acc_max·ds)
% =========================================================================
for i = 2:N
    ds = s_arc(i) - s_arc(i-1);    % 本段弧長
    if ds < 1e-6
        continue;   % 跳過重複點
    end
    % 由 i-1 點正推算 i 點的最大允許速度
    v_allow = sqrt(v_profile(i-1)^2 + 2*params.a_acc_max*ds);
    v_profile(i) = min(v_profile(i), v_allow);  % 取較小值
end

% 將最終速度規劃和弧長存回 refpath
refpath.v_profile = v_profile;  % 最終速度剖面 (m/s)
refpath.s_arc = s_arc;          % 弧長累積值 (m)

% =========================================================================
% 步驟 5.5：生成多條候選路徑（從 GPS waypoints）
% =========================================================================
% 將 refpath 的代表點抽稀為 GPS waypoints
stride = 50;  % 每 50 點取一個 waypoint
gps_wp = [refpath.x(1:stride:end), refpath.y(1:stride:end)];
path_candidates = my_multi_path(gps_wp, params.N_paths, params);

% 初始選擇中間那條（最接近原始路徑）
active_path_idx = ceil(params.N_paths / 2);
refpath_active  = path_candidates{active_path_idx};

% =========================================================================
% 步驟六：初始化車輛狀態
% 車輛初始狀態設定在路徑起點
% =========================================================================
x0   = refpath_active.x(1);          % tractor rear axle or control point
y0   = refpath_active.y(1);
yaw0 = refpath_active.phi(1);        % tractor heading
yaw1 = refpath_active.phi(1);        % trailer heading, 初始對齊
v    = refpath_active.v_profile(1);

% 初始 hitch / trailer axle
xh = x0 - params.M1 * cos(yaw0);
yh = y0 - params.M1 * sin(yaw0);
x1 = xh - params.L2 * cos(yaw1);
y1 = yh - params.L2 * sin(yaw1);

% 模擬總步數（最多 3000 步，或路徑長度，取較小值）
Nsim = min(length(refpath.x), 3000);

% 上一步的最近點索引，用於 controller 搜尋視窗加速
idx_prev = 1;

% =========================================================================
% 初始化歷史記錄結構（儲存每步模擬結果，用於事後分析與繪圖）
% =========================================================================
hist.v          = zeros(Nsim,1);  % 車輛速度 (m/s)
hist.delta      = zeros(Nsim,1);  % 前輪轉向角指令 (rad)
hist.idx_target = zeros(Nsim,1);  % 每步的 look-ahead 目標點索引
hist.idx_near   = zeros(Nsim,1);  % 每步的最近點索引
hist.Ld         = zeros(Nsim,1);  % 每步的動態前視距離 (m)
hist.alpha      = zeros(Nsim,1);  % 車輛到目標點的方位角誤差 (rad)
hist.cte        = zeros(Nsim,1);  % 橫向追蹤誤差 CTE (m)
hist.he         = zeros(Nsim,1);  % 航向誤差 (rad)
hist.kappa      = zeros(Nsim,1);  % 當步曲率指令 (1/m)
hist.a_lat      = zeros(Nsim,1);  % 當步側向加速度 (m/s²)

hist.theta0     = zeros(Nsim,1);  % 別名，讓 STEP4 animation 相容
hist.theta1     = zeros(Nsim,1);
hist.v_cmd      = zeros(Nsim,1);
hist.omega_cmd  = zeros(Nsim,1);
hist.err_y      = zeros(Nsim,1);
hist.Hx         = zeros(Nsim,1);
hist.Hy         = zeros(Nsim,1);
hist.active_idx = zeros(Nsim,1);  % 記錄每步實際使用的候選路徑編號

hist.x0         = zeros(Nsim,1);
hist.y0         = zeros(Nsim,1);
hist.yaw0       = zeros(Nsim,1);
hist.x1         = zeros(Nsim,1);
hist.y1         = zeros(Nsim,1);
hist.yaw1       = zeros(Nsim,1);
hist.xh         = zeros(Nsim,1);
hist.yh         = zeros(Nsim,1);

hist.v          = zeros(Nsim,1);
hist.delta      = zeros(Nsim,1);
hist.omega      = zeros(Nsim,1);
hist.idx_target = zeros(Nsim,1);
hist.idx_near   = zeros(Nsim,1);
hist.Ld         = zeros(Nsim,1);
hist.alpha      = zeros(Nsim,1);
hist.cte        = zeros(Nsim,1);
hist.he         = zeros(Nsim,1);
hist.kappa      = zeros(Nsim,1);
hist.a_lat      = zeros(Nsim,1);

% =========================================================================
% 步驟七：主模擬迴圈（Closed-Loop Bicycle Model Simulation）
% 每步流程：
%   1. 呼叫 pure_pursuit_controller 取得轉向角指令 delta
%   2. 執行速度跟隨（縱向控制）
%   3. 計算 CTE 和 Heading Error
%   4. Bicycle Model 狀態更新（Euler 積分）
%   5. 記錄本步資料
% =========================================================================

for k = 1:Nsim

        % ---- 即時路徑重選（每 T_replan 步一次）----
    if mod(k, params.T_replan) == 1
        trailer_state = [x0, y0, yaw0, yaw1, v];
        best_idx = select_best_path(path_candidates, trailer_state, params);
        if best_idx ~= active_path_idx
            active_path_idx = best_idx;
            refpath_active  = path_candidates{active_path_idx};
            idx_prev = 1;  % 重置搜尋索引
        end
    end

    hist.active_idx(k) = active_path_idx;

    if idx_prev > length(refpath_active.x) - 5
        idx_prev = max(1, length(refpath_active.x) - 10);  % 防止越界
    end

    % --- 呼叫 Pure Pursuit 控制器，取得轉向角指令 ---
    % 輸入：當前車輛狀態 (x, y, yaw, v)、參考路徑、參數、上步最近點索引
    % 輸出：delta（轉向角）、idx_target（目標點）、idx_near（最近點）
    %       Ld（前視距離）、alpha（方位角誤差）
    [delta, idx_target, idx_near, Ld, alpha] = ...
    pure_pursuit_controller(x0, y0, yaw0, v, refpath_active, params, idx_prev);

    % 更新搜尋視窗起點為本步最近點（下步搜尋從這裡開始，加速搜尋）
    idx_prev = idx_near;

    % --- 縱向速度跟隨控制 ---
    % 從速度規劃取得當前參考速度
    v_ref_now = refpath_active.v_profile(idx_near);

    % 一階速度跟隨：以 a_acc_max / a_dec_max 斜率趨近參考速度
    if v_ref_now > v
        % 需要加速：速度增量不超過 a_acc_max * Ts
        v = min(v + params.a_acc_max * params.Ts, v_ref_now);
    else
        % 需要減速：速度減量不超過 a_dec_max * Ts
        v = max(v - params.a_dec_max * params.Ts, v_ref_now);
    end

    % --- 計算追蹤誤差（用於記錄與事後分析）---
    x_ref   = refpath_active.x(idx_near);    % 最近參考點 x 座標
    y_ref   = refpath_active.y(idx_near);    % 最近參考點 y 座標
    yaw_ref = refpath_active.phi(idx_near);  % 最近參考點航向角

    dx = x0 - x_ref;  % 車輛相對參考點的 x 偏移
    dy = y0 - y_ref;  % 車輛相對參考點的 y 偏移

    % 橫向追蹤誤差 CTE（Lateral Error）
    % 定義：車輛位置在路徑法線方向的投影距離
    % 正值 = 車輛在路徑左側，負值 = 右側
    % 公式推導：將 (dx, dy) 投影到路徑法線方向 (-sin(yaw_ref), cos(yaw_ref))
    cte = -sin(yaw_ref)*dx + cos(yaw_ref)*dy;

    % 航向誤差 he（Heading Error）
    % 定義：車輛航向角與參考點切線方向的夾角
    % 使用 atan2(sin, cos) 確保結果在 (-π, π) 範圍內
    he = atan2(sin(yaw0 - yaw_ref), cos(yaw0 - yaw_ref));

    % --- 計算本步側向加速度（用於記錄）---
    kappa_now = tan(delta) / params.L1;  % 由轉向角估算曲率
    a_lat_now = v^2 * kappa_now;        % 側向加速度 = v² · kappa

   % --- Tractor 更新 ---
    x0  = x0  + v * cos(yaw0) * params.Ts;
    y0  = y0  + v * sin(yaw0) * params.Ts;
    yaw0 = yaw0 + (v / params.L1) * tan(delta) * params.Ts;
    yaw0 = atan2(sin(yaw0), cos(yaw0));

    % --- Hitch 點更新（tractor 後懸 M1 處）---
    xh = x0 - params.M1 * cos(yaw0);
    yh = y0 - params.M1 * sin(yaw0);

    % --- Trailer 航向更新（off-axle hitch 方程）---
    % omega1 = (v/L1)*tan(delta)  →  tractor yaw rate
    omega1 = (v / params.L1) * tan(delta);   % tractor yaw rate
    dphi = (v / params.L2) * sin(yaw0 - yaw1) ...
        - (params.M1 / params.L2) * omega1 * cos(yaw0 - yaw1);
    yaw1 = yaw1 + dphi * params.Ts;
    yaw1 = atan2(sin(yaw1), cos(yaw1));

    % --- Trailer 軸中心更新 ---
    x1 = xh - params.L2 * cos(yaw1);
    y1 = yh - params.L2 * sin(yaw1);
    % --- 記錄本步所有狀態 ---
    hist.v(k)          = v;
    hist.delta(k)      = delta;
    hist.idx_target(k) = idx_target;
    hist.idx_near(k)   = idx_near;
    hist.Ld(k)         = Ld;
    hist.alpha(k)      = alpha;
    hist.cte(k)        = cte;
    hist.he(k)         = he;
    hist.kappa(k)      = kappa_now;
    hist.a_lat(k)      = a_lat_now;

    hist.theta0(k)    = yaw0;
    hist.theta1(k)    = yaw1;
    hist.v_cmd(k)     = v;
    hist.omega_cmd(k) = (v / params.L1) * tan(delta);
    hist.err_y(k)     = cte;
    hist.Hx(k)        = xh;
    hist.Hy(k)        = yh;

    hist.x0(k)   = x0;
    hist.y0(k)   = y0;
    hist.yaw0(k) = yaw0;
    hist.x1(k)   = x1;
    hist.y1(k)   = y1;
    hist.yaw1(k) = yaw1;
    hist.xh(k)   = xh;
    hist.yh(k)   = yh;

end

% =========================================================================
% 步驟八：繪圖與輸出結果
% =========================================================================

% 存模擬結果給 STEP4_ETS2_Animation_MultiView.m
results.hist = hist;
results.ts   = (0:Nsim-1)' * params.Ts;
save('simulation_results.mat', 'results');
fprintf('✓ simulation_results.mat 已儲存\n');

% --- 圖一：路徑追蹤結果 ---
figure; hold on; axis equal; grid on;
plot(refpath.x, refpath.y, 'k:', 'LineWidth', 0.8, 'DisplayName', 'Reference (母路徑)');

colors = lines(params.N_paths);
for i = 1:params.N_paths
    if ~isempty(path_candidates{i})
        plot(path_candidates{i}.x, path_candidates{i}.y, '--', ...
             'Color', colors(i,:), 'HandleVisibility', 'off');
    end
end

for i = 1:params.N_paths
    mask = (hist.active_idx == i);
    if any(mask)
        scatter(hist.x0(mask), hist.y0(mask), 10, colors(i,:), 'filled', ...
            'DisplayName', sprintf('Tractor 使用 Path %d', i));
    end
end

plot(hist.x1, hist.y1, 'g-', 'LineWidth', 1.2, 'DisplayName', 'Trailer');
legend show;
title('Pure Pursuit Tracking + Path Switching');

% --- 圖二：轉向角指令歷程 ---
figure;
plot(rad2deg(hist.delta), 'LineWidth', 1.2);  % 轉換為角度顯示
grid on;
xlabel('Step');
ylabel('Steering Angle (deg)');
title('Steering Command');

% --- 圖三：追蹤誤差歷程（CTE 和 Heading Error）---
figure;
subplot(2,1,1);
plot(hist.cte, 'LineWidth', 1.2);   % 橫向誤差
grid on;
ylabel('CTE (m)');
title('Tracking Errors');

subplot(2,1,2);
plot(rad2deg(hist.he), 'LineWidth', 1.2);   % 航向誤差（轉角度顯示）
grid on;
xlabel('Step');
ylabel('Heading Error (deg)');

% --- 終端機輸出第一次誤差統計 ---
fprintf('CTE RMS = %.4f m\n',            rms(hist.cte));
fprintf('Heading Error RMS = %.4f deg\n', rms(rad2deg(hist.he)));
fprintf('Max |delta| = %.4f deg\n',       max(abs(rad2deg(hist.delta))));

% --- 圖四：速度規劃與側向加速度 ---
figure;
subplot(2,1,1);
plot(refpath.v_profile, 'r--', 'LineWidth', 1.2); hold on;  % 參考速度剖面
plot(hist.v, 'b-', 'LineWidth', 1.2);                        % 實際速度
grid on;
legend('Reference Speed Profile', 'Actual Speed');
ylabel('Speed (m/s)');
title('Speed Profile');

subplot(2,1,2);
plot(hist.a_lat, 'LineWidth', 1.2); hold on;
yline(params.a_lat_max, 'r--');   % 側向加速度上限
yline(-params.a_lat_max, 'r--'); % 側向加速度下限
grid on;
xlabel('Step');
ylabel('Lateral Accel (m/s^2)');
title('Lateral Acceleration');

switch_count = sum(diff(hist.active_idx) ~= 0);
fprintf('路徑切換次數 = %d 次（共 %d 步，平均每 %.1f 步切換一次）\n', ...
    switch_count, Nsim, Nsim/max(switch_count,1));

% --- 終端機輸出最終完整誤差統計 ---
fprintf('CTE RMS = %.4f m\n',            rms(hist.cte));
fprintf('Heading Error RMS = %.4f deg\n', rms(rad2deg(hist.he)));
fprintf('Max |delta| = %.4f deg\n',       max(abs(rad2deg(hist.delta))));
fprintf('Speed range = [%.4f, %.4f] m/s\n', min(hist.v), max(hist.v));
fprintf('Max |a_lat| = %.4f m/s^2\n',    max(abs(hist.a_lat)));
