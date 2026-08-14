% =========================================================================
% 檔案名稱: STEP1_VehicleParameters.m
% 版本   : Beta 1-5 V1.1（模組化整理版）
%
% 功能：
%   本檔案是整個模擬系統唯一的「參數中樞」。所有車輛幾何、運動學限制、
%   控制器增益、路徑生成/選路/安全機制的可調參數，全部集中在這裡定義，
%   最後統一存成 vehicle_params.mat，供其他所有 .m 檔案讀取使用。
%
%   這樣設計的目的是「模組化」：任何程式（不論是 Pure Pursuit、未來的
%   Frenet、或 MPC）都只需要 load('vehicle_params.mat','params')，
%   就能拿到一致的車輛模型與限制條件，不會有「這個檔案用一組參數、
%   那個檔案又用另一組參數」導致結果對不起來的問題。
%
% 使用方式：
%   1. 先執行本檔案（STEP1），產生 vehicle_params.mat
%   2. 再執行 STEP2_MultiPathGen.m（整條路線候選路徑預覽，非必要）
%   3. 再執行 main_pure_pursuit_sim.m（主模擬，會自動 load 本檔案存的參數）
%
% 重要提醒（容易混淆的地方）：
%   - 本檔案第 4 節設定 params.Ts = 0.02（50Hz），但 main_pure_pursuit_sim.m
%     會在載入參數「之後」把 params.Ts 覆寫成 0.05（20Hz）才拿去模擬。
%     這是因為 Pure Pursuit 追蹤迴圈用 20Hz 已經足夠、且能加快模擬速度；
%     0.02 這個值目前只有 MPC 相關的（尚未啟用的）計算會參考到。
%     若要修改「實際模擬用」的時間步長，請改 main_pure_pursuit_sim.m 裡的設定，
%     不要只改這裡。
%   - params.L2、params.trailer_L_total（現稱 trailer_length）意義不同：
%     L2 是「hitch 點到 trailer 後軸」的運動學距離（bicycle-trailer 模型
%     真正拿來算角度/位置的量），trailer_length 只是「貨櫃視覺外框全長」，
%     純粹給動畫畫車身用，兩者不能混用（这曾是舊版的一個混淆來源，
%     所以特別在這裡註明）。
% =========================================================================

clear; clc;
fprintf('=== STEP 1: 車輛與模擬參數初始化 ===\n');

params = struct();

% =========================================================================
% 第 1 節：車輛幾何參數（Pure Pursuit / Frenet / MPC 共用，運動學計算用）
% =========================================================================
% 座標系定義：以「拖車頭（tractor）後軸中心」為基準點 (x0, y0)，
% 車頭朝向角 yaw0 沿此點的行進方向定義。鉸接點（hitch/kingpin）在
% 拖車頭後軸中心「後方」M1 處；貨櫃（trailer）後軸中心則在鉸接點
% 「後方」再 L2 處。三者關係示意（由前到後）：
%
%     [拖車頭後軸, (x0,y0)] --M1--> [鉸接點, (xh,yh)] --L2--> [貨櫃後軸, (x1,y1)]
%
params.L1 = 4.5;         % [m] 拖車頭軸距（前軸到後軸距離）。決定拖車頭的
                          %     轉向幾何：前輪轉角 delta 造成的橫擺角速度為
                          %     omega1 = v/L1 * tan(delta)（標準 bicycle model）。
params.M1 = 1.0;         % [m] 鉸接點偏置：鉸接點在拖車頭後軸「後方」的距離。
                          %     M1=0 代表鉸接點就在後軸正上方（on-axle hitch，
                          %     教科書最簡化模型）；M1>0 才是真實聯結車常見的
                          %     off-axle hitch（鉸接點通常在後軸之後），
                          %     這會讓拖車頭轉向時多一項「鉸接點自身的
                          %     切線速度」需要投影到貨櫃航向方向，詳見
                          %     pure_pursuit_controller.m 呼叫端與主迴圈的
                          %     trailer 航向動力學推導。
params.L2 = 7.5;         % [m] 鉸接點到「貨櫃後軸」的距離 —— 這是貨櫃的
                          %     「運動學等效軸距」，決定貨櫃轉彎時的甩尾/
                          %     外偏（off-tracking）幅度：曲率 kappa 一定時，
                          %     穩態鉸接角 phi_ss ≈ atan(L2*kappa)，
                          %     穩態橫向外偏 ≈ L2*sin(phi_ss)。
                          %     L2 越長，同樣彎道下貨櫃甩得越開，這是真實
                          %     半掛卡車的物理特性，不是控制誤差
                          %     （detail 見 select_best_path.m 內
                          %     check_hitch_angle 的推導註解）。

% --- 1.1 車身寬度（運動學上不影響轉向計算，但選路評分/畫圖需要）---
params.w_tractor = 2.5;  % [m] 拖車頭車寬
params.w_trailer = 2.6;  % [m] 貨櫃車寬
params.tractor_width = params.w_tractor;
params.trailer_width = params.w_trailer;

% --- 1.2 車身外框（純視覺化用，畫車身矩形/3D模型時的前後懸長度） ---
% 這些「overhang（懸長）」定義車身輪廓比軸距多伸出多少，跟運動學計算
% (delta/omega/kappa 等) 完全無關，只有 STEP4 動畫畫車身矩形時會用到。
params.tractor_front_overhang = 1.4;   % [m] 拖車頭前軸再往前伸出的車頭長度
params.tractor_rear_overhang  = 0.8;   % [m] 拖車頭後軸再往後伸出的車尾長度
params.tractor_length = params.tractor_front_overhang + params.L1 + params.tractor_rear_overhang;

params.trailer_front_overhang = 1.2;   % [m] 貨櫃前緣到「鉸接點」的距離
params.trailer_rear_overhang  = 2.5;   % [m] 貨櫃後軸再往後伸出的車尾長度
params.trailer_length = params.trailer_front_overhang + params.L2 + params.trailer_rear_overhang;
params.trailer_L_total = params.trailer_length;   % 別名，相容舊版命名（=貨櫃全長，僅供視覺化）

params.kingpin_to_trailer_front = params.trailer_front_overhang;
params.kingpin_to_trailer_rear  = params.L2 + params.trailer_rear_overhang;

% =========================================================================
% 第 2 節：運動學物理限制（物理天花板，任何控制器都不能超過）
% =========================================================================
params.v_max = 50.0;          % [m/s] 車輛物理最高速（純上限，Pure Pursuit
                               %       實際巡航速度由 v_des 決定，遠低於此值）
params.v_min = -5.0;          % [m/s] 倒車限速（目前 Pure Pursuit 模擬不倒車，
                               %       此值保留給未來需要倒車操作的情境）
params.w_max = deg2rad(60);   % [rad/s] 拖車頭橫擺角速度（yaw rate）上限
params.phi_max = deg2rad(85); % [rad] 鉸接角（拖車頭與貨櫃航向夾角）上限，
                               %       超過這個角度視為「折疊/夾死（jackknife）」，
                               %       車輛已經無法再靠前進恢復幾何，是真正的
                               %       安全紅線。compute_hitch_speed_cap.m 與
                               %       hitch_angle_governor.m 都是圍繞這個
                               %       上限、在「還沒到之前」就先降速介入。
params.v_profile_min = 1.0;   % [m/s] 候選路徑 / 巡航速度剖面的下限，避免
                               %       速度規劃算出趨近於 0 的速度（會讓
                               %       Pure Pursuit 前視距離公式除以接近 0
                               %       的 v 而數值不穩定）。跟倒車限速 v_min
                               %       意義不同，不要混用。

% =========================================================================
% 第 3 節：縱向加速度 / Jerk 限制（目前僅供未來 MPC 使用，Pure Pursuit
%          實際用的是第 9 節的 a_acc_max / a_dec_max / a_lat_max）
% =========================================================================
params.acc_max = 1.0;         % [m/s^2] 最大加速度（MPC 用）
params.dec_max = -4.5;        % [m/s^2] 最大減速度（MPC 用，含負號）
params.jerk_max = 0.5;        % [m/s^3] 加加速度（Jerk）限制（MPC 用）

% =========================================================================
% 第 4 節：控制/模擬時間步長
% =========================================================================
params.Ts = 0.02;             % [s] 預設 50Hz。
                               %     ⚠ main_pure_pursuit_sim.m 會把這個值
                               %     覆寫成 0.05（20Hz）才真正拿去模擬，
                               %     這裡的 0.02 只有尚未啟用的 MPC 相關
                               %     計算會參考到，詳見檔案最上方的提醒。

% =========================================================================
% 第 5 節：LMI 設計參數（Linear Matrix Inequality，供未來 MPC/LPV 控制器
%          做穩定度驗證用，Pure Pursuit 完全不使用這一節）
% =========================================================================
params.lmi.x0_max = 1.5;          % [m] LMI 保證收斂域半徑（實際追蹤誤差
                                   %     遠小於此值，這只是理論上界）
params.lmi.safety_factor = 0.95;  % LMI 約束安全係數（實際限制仍以真實物理
                                   %     上限為準，這只是設計時多留的餘裕）
params.lmi.vr_min = 0.001;        % [m/s] LPV（線性參數變化）模型頂點 1：
                                   %       速度趨近 0 時的線性化基準速度
                                   %       （不能真的設 0，否則除以 vr 會發散）
params.lmi.vr_max = params.v_max; % [m/s] LPV 模型頂點 2：速度上限處的
                                   %       線性化基準速度

% =========================================================================
% 第 6 節：MPC 參數（供未來 MPC 控制器使用，Pure Pursuit 不使用）
% =========================================================================
params.mpc.Np = 30;           % 預測視窗長度（步數）
params.mpc.Q = [20, 20, 80];  % 狀態誤差權重 [縱向誤差, 橫向誤差, 航向誤差]
params.mpc.R = [5, 20];       % 控制輸入權重 [速度 v, 角速度 omega]
params.mpc.S = [10, 50];      % 控制輸入變化率（slew rate）權重，抑制轉向/速度指令抖動

% =========================================================================
% 第 7 節：相容性參數（沿用舊版 Beta 系列的命名，供舊腳本相容用）
% =========================================================================
params.d0 = params.M1;        % 別名：等同 M1，部分舊腳本用這個名字
params.T_preview = 1.5;       % [s] 舊版前視時間參數（目前 Pure Pursuit
                               %     改用「動態前視距離 Ld」公式，此值未使用）

% =========================================================================
% 第 8 節：多候選路徑產生器（Multi-Path Selector）參數
%          給 generate_local_paths.m / my_multi_path.m / select_best_path.m 用
% =========================================================================
params.lane_width    = 3.5;   % [m] 單一車道標準寬度。採用一般公路設計常用的
                               % 標準車道寬 3.5m（國際上多國公路設計手冊、
                               % AASHTO 等常見引用值多落在 3.5~3.75m 之間；
                               % 3.5m 是最常被引用的「標準值」）。若論文要引用
                               % 特定規範數值（例如大型車道/聯結車適用寬度，
                               % 常見到 3.75m），改這裡即可，其餘程式碼不用跟著改。
params.n_side_lanes  = 1;     % 候選路徑側向涵蓋範圍：往左右各延伸幾條「鄰車道」。
                               % 0 = 候選路徑只在「本車道內」微調（車道內閃避
                               % 小障礙物，例如坑洞、路上小雜物）；
                               % 1 = 候選路徑範圍延伸到左右各一條鄰車道，讓評分
                               % 機制在需要時（例如障礙物整條本車道都擋住）能
                               % 選到「換車道」的候選路徑，而不是被限制在本車道
                               % 內出不去。目前沒有障礙物成本項，這個範圍只是
                               % 讓候選路徑「有得選」，實際會不會選到鄰車道，
                               % 由 select_best_path.m 的評分結果決定（沒有障礙物
                               % 時，同車道置中候選路徑的 CTE 最小，一定會贏）。
params.N_paths       = 9;     % 候選路徑數量（在下方 candidate_span_half 的範圍內
                               % 等間距排列，需求是 >=3）。配合 n_side_lanes=1 時，
                               % 9 條剛好讓本車道跟左右鄰車道各分到約 3 條候選路徑
                               % （細部密度足夠做車道內微調，也涵蓋鄰車道位置）。
params.T_replan      = 10;    % 每隔幾個模擬步重新生成一次候選路徑並重選
                               % （T_replan * Ts = 10 * 0.05 = 0.5 秒一次，
                               % 這就是「即時」的更新頻率：車輛每移動 0.5 秒
                               % 就會以「當下位置」為錨點，重新生成一批新的
                               % 候選路徑，而不是整趟模擬只算一次）
params.w_cte         = 1.0;   % 候選路徑評分：橫向誤差（CTE）成本權重
params.w_kappa       = 2.0;   % 候選路徑評分：曲率成本權重（聯結車對曲率
                               % 更敏感，因為曲率直接關係到貨櫃甩尾幅度）
params.w_hitch       = 5.0;   % 候選路徑評分：目前鉸接角懲罰權重（鉸接角
                               % 越大，代表車輛當下越接近折疊，評分懲罰越重）

% --- 8.1 即時局部路徑生成參數（generate_local_paths.m 專用）---
% 這是這次新增、把「候選路徑生成」從「整趟路線只算一次」改成「每次
% replan 都以車輛當下位置為錨點、重新生成」的關鍵參數，詳細原理見
% generate_local_paths.m 檔頭註解。
params.local_horizon_m     = 35;  % [m] 每次 replan 往「前方」取的母路徑
                                   %     窗口長度。只在這個窗口內生成候選
                                   %     路徑，而不是對整條路線（可能上百
                                   %     公尺甚至上千公尺）都重算一次，
                                   %     這樣才能在 0.5 秒的 replan 週期內
                                   %     即時算完。
params.local_wp_spacing    = 1.5; % [m] 窗口內 waypoint 的抽稀間距。這個值
                                   %     太粗（間距太大）會讓 my_path() 逐段
                                   %     擬合時的目標航向角估計變得不準，
                                   %     擬合出過度彎曲、不平滑的路徑；
                                   %     太細則計算量增加（segment 數量
                                   %     跟 1/spacing 成正比）。1.5m 是
                                   %     實測後兼顧平滑度與即時性的折衷值。
params.local_sample_stride = 25;  % my_path() 每段內部用 1000 個弧長取樣點
                                   %     描述曲線（見 my_dynamic.m），這裡
                                   %     每隔 25 點取一個，把每段密度降到
                                   %     約 40 點，避免候選路徑陣列過大。

% --- 8.2 鉸接角安全機制參數 ---
% compute_hitch_speed_cap.m（規劃階段：曲率預估限速）與
% hitch_angle_governor.m（執行階段：即時監控降速）共用同一組「以
% phi_max 為基準、取不同比例當門檻」的設計語言，兩者互為前後兩道防線：
% 前者在路徑規劃時就先避開太尖銳的彎道，後者則是萬一動態追蹤過程中
% 鉸接角仍然逼近上限時的最後防護網。
params.hitch_speed_cap_frac = 0.70;  % compute_hitch_speed_cap：預估鉸接角
                                      % 低於 phi_max*0.70 時完全不限速；
                                      % 超過才開始依比例降速。
params.hitch_speed_cap_gain = 4.0;   % 超過門檻後的降速強度（越大降得越快）
params.hitch_gov_warn_frac  = 0.60;  % hitch_angle_governor：即時鉸接角低於
                                      % phi_max*0.60 時不介入；超過開始降速。
params.hitch_gov_hard_frac  = 0.90;  % 即時鉸接角達到 phi_max*0.90 時，
                                      % 降速到 v_profile_min（最低速度）。

% =========================================================================
% 第 9 節：速度規劃參數（給 compute_v_profile.m / my_multi_path.m /
%          main_pure_pursuit_sim.m 的曲率限速與 forward-backward pass 用）
% =========================================================================
params.v_des     = 6.0;    % [m/s] 期望巡航速度（無曲率限制時的目標速度）
params.a_lat_max = 2.0;    % [m/s^2] 側向加速度上限，決定曲率限速：
                            %         v_curve = sqrt(a_lat_max / |kappa|)
params.a_acc_max = 1.0;    % [m/s^2] 縱向加速度上限（forward pass 用）
params.a_dec_max = 1.5;    % [m/s^2] 縱向減速度上限（backward pass 用，
                            %         確保在到達急彎前已經來得及減速）

% =========================================================================
% 儲存所有參數
% =========================================================================
save('vehicle_params.mat', 'params');

fprintf('✓ 參數已儲存 (vehicle_params.mat)\n');
fprintf('  - 車輛幾何: L1=%.1fm(拖車頭軸距) M1=%.1fm(鉸接偏置) L2=%.1fm(貨櫃軸距)\n', ...
    params.L1, params.M1, params.L2);
fprintf('  - 物理速限: %.1f m/s (%.0f km/h)，巡航速度 v_des=%.1f m/s\n', ...
    params.v_max, params.v_max*3.6, params.v_des);
fprintf('  - 鉸接角上限 phi_max=%.1f°（安全網介入點: %.1f°~%.1f°）\n', ...
    rad2deg(params.phi_max), rad2deg(params.phi_max)*params.hitch_gov_warn_frac, ...
    rad2deg(params.phi_max)*params.hitch_gov_hard_frac);
fprintf('  - 即時局部路徑生成: 每 %.2fs（T_replan=%d 步）重生成 %d 條候選路徑，窗口 %.0fm\n', ...
    params.T_replan*0.05, params.T_replan, params.N_paths, params.local_horizon_m);
fprintf('  - [MPC/LMI 參數已預留，Pure Pursuit 階段尚未使用]\n');
fprintf('\n請執行 main_pure_pursuit_sim.m 進行 Pure Pursuit 模擬\n');
