% =========================================================================
% 檔案名稱: generate_local_paths.m
%
% 功能：這是「即時多候選路徑生成」的核心模組。每次被呼叫，都會：
%       (1) 用車輛「目前實際位置」在母路徑上找錨點，
%       (2) 只在錨點前方一小段窗口（local_horizon_m，預設 35m）內，
%       (3) 重新生成 N_paths 條側向平行的候選路徑。
%       main_pure_pursuit_sim.m 每隔 T_replan 步（預設 0.5 秒）就呼叫一次
%       本函式，所以候選路徑會隨著車輛前進，不斷從「車輛當下所在的地方」
%       重新長出來 —— 這就是「即時路徑生成」的實際做法。
%
% ─────────────────────────────────────────────────────────────────────
% 舊架構 vs. 新架構（回答「怎麼做到即時生成」這個問題）：
%
%   【舊架構】main_pure_pursuit_sim.m 在模擬迴圈「開始前」呼叫一次
%   my_multi_path(gps_wp, ...)，gps_wp 是把「整條母路徑」（可能上百公尺、
%   上千個點）抽稀出來的全部 waypoints。my_multi_path.m 會對這整條路線
%   一次性跑完 shift_waypoints_lateral -> stitch_local_path -> 曲率/
%   速度規劃，產生 5 條「從路線起點到路線終點」的候選路徑，之後整趟
%   模擬都只在這 5 條固定的路徑之間做選擇（select_best_path.m 每隔
%   T_replan 步重新「打分數」，但打分數的對象永遠是同樣這 5 條路徑）。
%   這樣的問題是：候選路徑在模擬開始那一刻就固定死了，車輛移動到路線
%   任何位置，看到的都是「同一批」候選路徑，沒辦法因應車輛「當下」
%   周遭環境（例如之後要加入的路障）動態調整候選路徑的形狀或位置。
%
%   【新架構】本函式 generate_local_paths.m 不吃「整條路線的 waypoints」，
%   而是吃車輛的「即時狀態」state=[x,y,yaw]。main_pure_pursuit_sim.m 每次
%   呼叫時都傳入車輛「這一刻」的真實位置，本函式內部重新對母路徑做一次
%   最近點搜尋（步驟1），只取車輛前方一小段窗口（步驟2），在這段窗口內
%   重新產生候選路徑（步驟3、4）。所以車輛每前進 0.5 秒，看到的候選路徑
%   都是「以剛剛量到的新位置」為起點重新生成的一批新路徑，不是模擬開始
%   時就寫死的固定路徑。這正是未來加入路障閃避的基礎：只要在候選路徑
%   評分（select_best_path.m）裡加入「跟障礙物距離」的懲罰項，車輛就能
%   在每次重新生成候選路徑時，自動避開當下偵測到的障礙物，而不用等到
%   下一次整趟路線重新規劃。
%
%   【底層數學引擎沒有變】不論新舊架構，實際「怎麼把離散 waypoints
%   擬合成平滑曲線」這件事，都是呼叫同一套 my_path.m / my_dynamic.m
%   Newton-Raphson 曲率多項式引擎（完全沒有修改，見 my_path.m 檔頭的
%   詳細說明）。新舊架構的差別純粹是「餵給這套引擎的 waypoints 是整條
%   路線、還是車輛前方一小段窗口」，以及「呼叫的時間點是模擬開始前
%   一次性、還是每 0.5 秒車輛移動後重新呼叫」，屬於呼叫端架構的改動，
%   不是底層數學的改動。
% ─────────────────────────────────────────────────────────────────────
%
% 純函式，不依賴全域變數／檔案 I/O，之後 Frenet/MPC 可直接複用（只要
% 有車輛狀態 state、母路徑 refpath、params，任何控制方法都能呼叫本函式
% 拿到一組候選路徑）。
%
% 輸入：
%   state     : [x, y, yaw] 車輛目前位姿（錨點，這是「即時性」的來源）
%   refpath   : 母路徑 struct，含 .x .y .phi（環形路線，首尾相接）
%   params    : 需要 lane_width, n_side_lanes（候選路徑側向涵蓋左右各幾條
%               鄰車道，0=只在本車道內）, local_horizon_m, local_wp_spacing,
%               local_sample_stride, a_lat_max/a_acc_max/a_dec_max/v_des/
%               v_profile_min, L1, L2, phi_max, hitch_speed_cap_frac/gain
%   N_paths   : 候選路徑數量（可調，需求是 >=3）
%   obstacles : 保留參數，目前傳 [] / {}；之後路障功能直接接這個介面，不必再改函式簽名
%
% 輸出：
%   path_candidates : cell array，每個元素是 refpath 結構 (.x .y .phi .kappa .v_profile)，
%                      格式與 pure_pursuit_controller.m / select_best_path.m 現有介面相同
%
% 演算法步驟：
%   1. 對母路徑做全域最近點搜尋，找到車輛目前位置的錨定索引
%   2. 從錨點沿母路徑往前走，累積弧長直到達到 local_horizon_m（窗口取樣）
%   3. 把窗口內的點依 local_wp_spacing 抽稀成局部 waypoints（含航向角內插）
%   4. 對每個側向偏移量呼叫 shift_waypoints_lateral + stitch_local_path，
%      生成一條候選路徑，再算曲率與速度剖面

function path_candidates = generate_local_paths(state, refpath, params, N_paths, obstacles) %#ok<INUSD>
    Nref = length(refpath.x);

    % ---- 1. 全域最近點搜尋，找車輛在母路徑上的錨定索引 ----
    dist_all = hypot(refpath.x - state(1), refpath.y - state(2));
    [~, idx_near] = min(dist_all);

    % ---- 2. 從錨定索引往前走，累積弧長直到涵蓋 local_horizon_m（環形路線用 mod 處理接縫）----
    idx_list = idx_near;
    arc = 0;
    i_cur = idx_near;
    while arc < params.local_horizon_m && numel(idx_list) < Nref
        i_nxt = mod(i_cur, Nref) + 1;
        arc = arc + hypot(refpath.x(i_nxt)-refpath.x(i_cur), refpath.y(i_nxt)-refpath.y(i_cur));
        idx_list(end+1) = i_nxt; %#ok<AGROW>
        i_cur = i_nxt;
    end

    window_x   = refpath.x(idx_list);
    window_y   = refpath.y(idx_list);
    window_phi = unwrap(refpath.phi(idx_list));   % 沿窗口 unwrap，避免跨 -pi/+pi 內插出錯

    % 濾掉近乎重複的點，避免下面 interp1 對重複弧長座標出錯
    d_step = hypot(diff(window_x), diff(window_y));
    keep = [true; d_step > 1e-9];
    window_x   = window_x(keep);
    window_y   = window_y(keep);
    window_phi = window_phi(keep);

    % ---- 3. 依 local_wp_spacing 抽稀成局部 waypoints ----
    % 航向直接內插母路徑既有、平滑的 refpath.phi（而不是用稀疏點的 gradient() 重新估計），
    % 避免粗抽稀造成航向估計雜訊、進而讓 my_path() 擬合出過度彎曲的路徑
    s_win = [0; cumsum(hypot(diff(window_x), diff(window_y)))];
    s_targets = (0:params.local_wp_spacing:s_win(end))';
    if isempty(s_targets) || s_targets(end) < s_win(end)
        s_targets(end+1,1) = s_win(end);
    end
    local_x   = interp1(s_win, window_x,   s_targets, 'linear');
    local_y   = interp1(s_win, window_y,   s_targets, 'linear');
    local_phi = interp1(s_win, window_phi, s_targets, 'linear');
    local_wp = [local_x, local_y];
    phi_wp   = local_phi;

    % 注意：窗口起點刻意「不」強制改成車輛的精確瞬時位置/航向。
    % 起點就是母路徑上離車輛最近的點（第 1 步的全域最近點搜尋已保證很接近車輛實際位置，
    % 差距通常就是當下的 CTE，一般是次公尺級），讓每段路徑都能保持平滑銜接；
    % 若強制讓第一段在很短的弧長內同時修正位置與航向誤差，會逼出接近物理極限的曲率尖峰，
    % 這件事本身應該交給追蹤控制器（look-ahead 內的漸進修正）處理，不該由路徑幾何硬扛。
    % 「即時、以車輛當前位置為錨點重新生成」的需求，靠每次 replan 都重新做最近點搜尋來達成。

    % ---- 4. 產生 N_paths 條側向偏移候選路徑 ----
    % 側向偏移範圍不再侷限於「本車道」，而是往左右各延伸 n_side_lanes 條
    % 鄰車道（n_side_lanes=0 則退回只在本車道內微調）。這樣候選路徑集合
    % 同時涵蓋「車道內小幅閃避」跟「換車道閃避」兩種選項，實際會不會選到
    % 鄰車道由 select_best_path.m 的評分結果決定：沒有障礙物時，本車道
    % 置中候選路徑的 CTE 最小、一定會贏，行為跟只有本車道時完全一樣；
    % 未來加入障礙物成本後，本車道被擋住時評分機制才會自然轉向鄰車道的
    % 候選路徑，不需要另外寫一套「要不要換車道」的判斷邏輯。
    n_side_lanes = 0;
    if isfield(params, 'n_side_lanes'), n_side_lanes = params.n_side_lanes; end
    span_half = params.lane_width/2 + n_side_lanes * params.lane_width;
    offsets = linspace(-span_half, span_half, N_paths);
    path_candidates = cell(1, N_paths);
    win = 9;   % 與 compute_path_curvature 的平滑視窗一致，供防呆用

    for i = 1:N_paths
        wp_shifted = shift_waypoints_lateral(local_wp, offsets(i));

        [path_x, path_y, path_phi] = stitch_local_path(wp_shifted, phi_wp, params.local_sample_stride);

        if isempty(path_x) || length(path_x) < win
            path_candidates{i} = [];
            continue;
        end

        cand.x = path_x;
        cand.y = path_y;
        cand.phi = path_phi;
        cand.kappa = compute_path_curvature(path_x, path_y, path_phi, params);
        cand.v_profile = compute_v_profile(cand.kappa, params);
        path_candidates{i} = cand;
    end
end
