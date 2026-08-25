% =========================================================================
% 檔案名稱: my_multi_path.m
%
% 功能：「整條路線一次性」生成 N 條側向平移候選路徑，主要供
%       STEP2_MultiPathGen.m 做整條路線的候選路徑預覽圖用（純視覺化，
%       不參與 main_pure_pursuit_sim.m 的即時模擬）。
%
%       這是本專案「多候選路徑」機制最早的版本：對整條母路徑
%       （gps_waypoints，可能上百公尺）一次跑完側向平移＋逐段擬合＋
%       曲率/速度規劃，產生 N 條從起點到終點的完整候選路徑。
%
%       這次修改後，本函式不再自己維護一份「側向平移／逐段拼接／
%       曲率／速度規劃」的邏輯，而是呼叫跟 generate_local_paths.m
%       （即時局部生成）完全相同的四個共用模組：
%           shift_waypoints_lateral.m → stitch_local_path.m
%               → compute_path_curvature.m → compute_v_profile.m
%       這樣兩處只需要維護同一套核心邏輯，不會有「改了即時版卻忘記
%       改整條路線版」導致兩邊行為分岔的風險（這正是模組化的目的）。
%       也因為呼叫的是同一套 stitch_local_path.m，本函式自動繼承了
%       這次修正的轉置 bug 修正（見 stitch_local_path.m 檔頭說明），
%       不需要額外處理。
%
% 跟 generate_local_paths.m 的差異，只有兩點：
%   1. 餵進去的 waypoints 是「整條路線」而不是「車輛前方一小段窗口」
%   2. 只在被呼叫的當下執行一次（STEP2 預覽腳本手動執行），不像
%      generate_local_paths.m 是模擬迴圈裡每隔 T_replan 步自動重新呼叫
%
% 輸入：
%   gps_waypoints : Nx2 矩陣 [x, y]，整條母路徑抽稀後的 waypoints
%   N_paths       : 候選路徑數量
%   params        : 需要 lane_width, n_side_lanes（候選路徑側向涵蓋左右各幾條
%                   鄰車道，0=只在本車道內）, L1, a_lat_max, a_acc_max,
%                   a_dec_max, v_des, v_profile_min, L2, phi_max,
%                   hitch_speed_cap_frac/gain
%
% 輸出：
%   path_candidates : cell array，每個元素是 refpath 結構 (x, y, phi, kappa, v_profile)
%   offsets         : 1xN_paths，第 i 條候選路徑實際用到的側向偏移量（呼叫端
%                      需要這個數值時直接用這個輸出，不要自己反推公式——
%                      STEP2_MultiPathGen.m 曾經各自維護一份同樣的公式，
%                      兩邊沒同步更新導致標示數值跟實際生成的候選路徑對不起來）
% =========================================================================

function [path_candidates, offsets] = my_multi_path(gps_waypoints, N_paths, params)
    win = 9;              % 供防呆用，需與 compute_path_curvature 的平滑視窗一致
    sample_stride = 25;   % 每段 1000 個取樣點降到 ~40 點，整條路線預覽仍平滑但不會產生過量資料

    path_candidates = cell(1, N_paths);
    % 側向偏移範圍涵蓋左右各 n_side_lanes 條鄰車道，跟 generate_local_paths.m
    % 用同一套邏輯（見該檔案「產生 N_paths 條側向偏移候選路徑」段落的說明）
    n_side_lanes = 0;
    if isfield(params, 'n_side_lanes'), n_side_lanes = params.n_side_lanes; end
    span_half = params.lane_width/2 + n_side_lanes * params.lane_width;
    offsets = linspace(-span_half, span_half, N_paths);

    % 從 GPS waypoints 估算每一點的初始航向角（中心線的切線方向）
    dx_wp = gradient(gps_waypoints(:,1));
    dy_wp = gradient(gps_waypoints(:,2));
    phi_wp = atan2(dy_wp, dx_wp);

    for i = 1:N_paths
        % 側向平移出第 i 條候選路徑的 waypoints
        wp_shifted = shift_waypoints_lateral(gps_waypoints, offsets(i));

        % 逐段用 my_path() 擬合、拼接成平滑曲線（含轉置修正，見 stitch_local_path.m）
        [path_x, path_y, path_phi] = stitch_local_path(wp_shifted, phi_wp, sample_stride);

        if isempty(path_x) || length(path_x) < win
            path_candidates{i} = [];   % 標記此候選路徑無效（該段 Newton-Raphson 全數不收斂）
            continue;   % 跳過這條路徑，直接進入下一個 i
        end

        % 存成 refpath 結構，並計算曲率與速度規劃
        cand.x = path_x;
        cand.y = path_y;
        cand.phi = path_phi;
        cand.kappa = compute_path_curvature(path_x, path_y, path_phi, params);
        cand.v_profile = compute_v_profile(cand.kappa, params);
        path_candidates{i} = cand;
    end
end
