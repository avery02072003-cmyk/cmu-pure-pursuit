% =========================================================================
% 檔案名稱: stitch_local_path.m
%
% 功能：把一串 waypoints（可能是原始的，也可能是 shift_waypoints_lateral.m
%       側向平移過的）逐段用 my_path() 擬合成平滑曲線，再拼接成一條完整
%       路徑。這是「離散 waypoints」→「平滑、曲率連續的候選路徑」的
%       轉換核心，my_multi_path.m（整條路線版）與 generate_local_paths.m
%       （即時局部版）都呼叫本函式做同一件事，差別只在傳進來的
%       waypoints 是「整條路線的」還是「車輛前方一小段窗口的」。
%
% 座標轉換原理（世界座標 ↔ 局部座標）：
%   my_path(x_tag, y_tag, phi_tag, xic) 只認得「局部座標系」：以該段起點
%   為原點、起點航向為 x 軸正方向。但 waypoints 存的是世界座標，所以
%   每一段都要先把「終點相對起點的世界座標位移」轉換成局部座標系下的
%   座標，等 my_path() 擬合完再轉換回世界座標：
%
%     世界座標 -> 局部座標（用該段起點航向 phi_start 把座標軸「轉正」）：
%       dx_world = x_{j+1} - x_j,  dy_world = y_{j+1} - y_j
%       [x_tag; y_tag] = R(-phi_start) * [dx_world; dy_world]
%       其中 R(theta) = [cos theta, -sin theta; sin theta, cos theta]
%       （R(-phi_start) 就是程式碼裡 c=cos(-phi_start), s=sin(-phi_start)
%        算出來的 2x2 旋轉矩陣，把世界座標的位移「轉回」局部座標系）
%       phi_tag = 終點航向 - 起點航向（並用 atan2(sin,cos) 包回 -pi~pi，
%       避免角度跨越 ±180° 時算出錯誤的目標轉角）
%
%     局部座標 -> 世界座標（my_path() 擬合出來的曲線點，做反向旋轉 + 平移）：
%       [x_world; y_world] = R(phi_start) * [x_local; y_local] + [x_j; y_j]
%       phi_world = phi_local + phi_start
%
%   這樣不論這一段路徑實際朝向哪個方向，my_path() 內部永遠只需要解
%   「從原點朝向 0 度出發」這一種標準化問題，大幅簡化 Newton-Raphson
%   要處理的情境。
%
% ⚠ 轉置修正說明（這是這次修正 bug 的關鍵所在）：
%   my_path() 回傳的 seg 是 4xM 矩陣（列＝[x;y;theta;kappa] 四個狀態分量，
%   欄＝沿弧長的 M=1000 個取樣點，見 my_dynamic.m 檔頭說明）。
%   本函式呼叫 my_path() 後，第一件事就是 seg = seg'，把它轉置成
%   Mx1000 → 1000x4，之後 seg(:,1)/(:,2)/(:,3) 才會正確對應到
%   「x 座標欄／y 座標欄／航向角欄」（各 1000x1）。
%   舊版 my_multi_path.m 沒有做這個轉置，seg(:,1) 誤取到「第 1 個取樣點
%   的四維狀態向量」（因為 xic=[0,0,0,0]，第1個取樣點必為 [0;0;0;0]），
%   seg(:,2) 誤取到「第 2 個取樣點的四維狀態向量」，導致每段路徑只
%   算出 4 個幾乎重疊在原點附近的點，路徑因此退化成雜訊、曲率估計
%   完全失真。這正是本次修正的兩個核心 bug（路徑一直卡在同一條候選路徑、
%   貨櫃橫向偏移過大）背後真正的根源，詳見 select_best_path.m 與
%   generate_local_paths.m 檔頭的完整說明。
%
% 降採樣（sample_stride）：
%   my_path() 每段都展開成 1000 個密集取樣點，如果整條路徑有幾十甚至
%   上百段（例如整條路線一次性生成時），全部串起來會產生數十萬個點，
%   既浪費記憶體也拖慢後續曲率估計（compute_path_curvature.m 的
%   gradient() 運算量正比於點數）。這裡用 seg(1:sample_stride:end, :)
%   每隔 sample_stride 個點取一個，把每段密度降到合理範圍（預設
%   sample_stride=25 時，每段約 40 點）。
%
% 容錯處理（try/catch）：
%   my_path() 內部 Newton-Raphson 在某些幾何條件下可能無法收斂（見
%   my_path.m 的「發散」錯誤），這種情況下本函式直接跳過該段（continue），
%   不把這段加入輸出路徑。呼叫端（my_multi_path.m / generate_local_paths.m）
%   若因此導致整條候選路徑點數過少，會在自己的邏輯裡把該候選路徑標記
%   為無效（回傳空陣列 []），下游的 select_best_path.m 則會自動跳過
%   無效的候選路徑，不會影響其他候選路徑的正常運作。
%
% 輸入：
%   wp_shifted    : Nx2 矩陣，這一條候選路徑的 waypoints（世界座標，
%                   已經過 shift_waypoints_lateral.m 側向平移，或就是
%                   原始中心線 waypoints）
%   phi_wp        : Nx1 向量，每個 waypoint 的目標航向角（世界座標系），
%                   通常是「中心線」（平移前）的航向角，因為側向平移
%                   量遠小於路徑曲率半徑，平移後的切線方向變化可忽略
%   sample_stride : 每段 1000 個取樣點的降採樣間隔
%
% 輸出：
%   path_x, path_y, path_phi : 拼接後的完整路徑座標與航向角（世界座標系）
% =========================================================================

function [path_x, path_y, path_phi] = stitch_local_path(wp_shifted, phi_wp, sample_stride)
    path_x = []; path_y = []; path_phi = [];
    xic = [0, 0, 0, 0];   % 每段都從局部原點、局部朝向 0、局部曲率 0 開始
                           % （曲率 0 是刻意的：讓相鄰兩段銜接處曲率連續，
                           %   不會出現轉向瞬間跳變，因為 my_path() 也要求
                           %   終點曲率固定為 0，見 my_path.m）

    for j = 1:size(wp_shifted,1)-1
        % ---- 世界座標系下，這一段的起點到終點位移 ----
        dx_world = wp_shifted(j+1,1) - wp_shifted(j,1);
        dy_world = wp_shifted(j+1,2) - wp_shifted(j,2);

        phi_start = phi_wp(j);   % 這一段的「局部座標系」基準朝向（起點航向）

        % ---- 世界座標 -> 局部座標：用 R(-phi_start) 把位移轉到「起點朝向=0」的座標系 ----
        c = cos(-phi_start); s = sin(-phi_start);
        x_tag = c*dx_world - s*dy_world;
        y_tag = s*dx_world + c*dy_world;

        % ---- 目標航向角改為「相對變化量」（終點航向 - 起點航向），並包回 -pi~pi ----
        phi_tag = phi_wp(j+1) - phi_start;
        phi_tag = atan2(sin(phi_tag), cos(phi_tag));

        try
            [seg, ~] = my_path(x_tag, y_tag, phi_tag, xic);

            seg = seg';                          % 修正：4xM -> Mx4（列=取樣點，欄=[x,y,theta,kappa]）
            seg = seg(1:sample_stride:end, :);    % 降採樣，控制候選路徑點數密度

            % ---- 局部座標 -> 世界座標：用 R(phi_start) 轉回去、再平移到 wp_shifted(j,:) ----
            c2 = cos(phi_start); s2 = sin(phi_start);
            seg_x_world = c2*seg(:,1) - s2*seg(:,2) + wp_shifted(j,1);
            seg_y_world = s2*seg(:,1) + c2*seg(:,2) + wp_shifted(j,2);
            seg_phi_world = seg(:,3) + phi_start;

            path_x   = [path_x;   seg_x_world];
            path_y   = [path_y;   seg_y_world];
            path_phi = [path_phi; seg_phi_world];

            xic = [0, 0, 0, 0];   % 下一段永遠從局部原點、局部朝向0、曲率0重新開始
        catch
            continue;   % 此段 Newton-Raphson 不收斂（幾何不可行），跳過不採用
        end
    end
end
