% =========================================================================
% 檔案名稱: STEP1_VehicleParameters.m
% 版本: Beta 1-5 V1.0 (LMI+MPC Baseline)
% 功能:
%   1. [新增] LMI 設計參數 (x0_max, safety_factor)
%   2. [保留] Beta 1-3 所有物理參數
%   3. [準備] 為 acc/jerk 約束預留參數
% =========================================================================

clear; clc;
fprintf('=== STEP 1: Beta 1-5 參數初始化 ===\n');

params = struct();

% --- 1. 車輛幾何參數 (與 Beta 1-3 相同) ---
params.L1 = 4.5;        % [m] 車頭軸距
params.w_tractor = 2.5; % [m] 車寬
params.M1 = 1.0;        % [m] 鉸接點偏置

params.L2 = 10.0;       % [m] 貨櫃長度
params.w_trailer = 2.6; % [m] 貨櫃寬度

% 在 STEP1_VehicleParameters.m 補這行
params.d_hitch2axle = 7.5;   % hitch 點到 trailer 後軸的距離 (m)
                              % 通常 = L2 * 0.75，視實際車型調整

% [視覺化參數]
params.tractor_length = params.L1 + 1.5;
params.trailer_length = params.L2 + 2.0;
params.tractor_width = params.w_tractor;
params.trailer_width = params.w_trailer;

% --- 2. 運動學限制 (物理天花板) ---
params.v_max = 50.0;         % [m/s] 物理最高速
params.v_min = -5.0;         % [m/s] 倒車限速
params.w_max = deg2rad(60);  % [rad/s] 角速度限制）
params.phi_max = deg2rad(85);% [rad] 最大折疊角

% --- 3. 加速度/Jerk 限制 (新增給 MPC 用) ---
params.acc_max = 1.0;        % [m/s^2] 最大加速度
params.dec_max = -4.5;       % [m/s^2] 最大減速度
params.jerk_max = 0.5;       % [m/s^3] Jerk 限制

% --- 4. 控制週期 ---
params.Ts = 0.02;            % [s] 50Hz (Beta 1-3 保持)

% --- 5. LMI 設計參數 (調整以通過求解，不影響實際物理限制) ---
params.lmi.x0_max = 1.5;          % LMI 保證域半徑 (實際誤差遠小於此)
% params.lmi.vr_nominal = 50;       % LMI 線性化標稱速度 (接近巡航速度)
params.lmi.safety_factor = 0.95;  % LMI 約束安全係數 (MPC 內仍用真實限制)

% 改為（新增）：
params.lmi.vr_min      = 0.001;            % LPV 頂點 1：vr 不能為 0
params.lmi.vr_max      = params.v_max;  % LPV 頂點 2：vr = 50 m/s

% --- 6. MPC 參數 (新增) ---
params.mpc.Np = 30;          % 預測視窗 (從 30 降到 10)
params.mpc.Q = [20, 20, 80]; % 狀態權重 [ex, ey, eθ]
params.mpc.R = [5, 20];      % 控制權重 [v, ω]
params.mpc.S = [10, 50];     % Slew rate 權重

% --- 7. 相容性參數 ---
params.d0 = params.M1;
params.T_preview = 1.5;

% 儲存
save('vehicle_params.mat', 'params');

fprintf('✓ 參數已儲存 (vehicle_params.mat)\n');
fprintf('  - 物理速限: %.1f m/s (%.0f km/h)\n', params.v_max, params.v_max*3.6);
fprintf('  - 加速度限制: [%.1f, %.1f] m/s^2\n', params.dec_max, params.acc_max);
fprintf('  - Jerk 限制: %.1f m/s^3\n', params.jerk_max);
fprintf('  - LMI 保證域: %.1f m\n', params.lmi.x0_max);
fprintf('  - MPC 視窗: %d 步 (%.2f 秒)\n', params.mpc.Np, params.mpc.Np*params.Ts);
fprintf('\n請執行 STEP2 (保持 Beta 1-3 版本) 產生路徑\n');
