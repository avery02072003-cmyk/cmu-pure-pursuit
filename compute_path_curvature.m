% =========================================================================
% 檔案名稱: compute_path_curvature.m
%
% 功能：由一串路徑點的座標 (x,y) 與航向角 phi，數值估計每一點的曲率
%       kappa，並做物理可行性截斷與平滑化。這是候選路徑產生流程的
%       最後一步：有了 kappa 之後，compute_v_profile.m 才能據此規劃
%       每一點允許的最高速度。
%
% 數學原理（曲率的定義）：
%       kappa(s) = d(theta)/ds
%   即「航向角對弧長的變化率」。用離散資料估計時：
%       ds_i  ≈ hypot(dx_i, dy_i)          （相鄰點間的弧長，用 gradient 估計）
%       kappa_i ≈ d(unwrap(phi))_i / ds_i  （航向角變化量 / 弧長變化量）
%
%   用 unwrap(phi) 而不是直接對 phi 做 gradient，是因為 phi 是用
%   atan2 算出來、被限制在 (-pi, pi] 範圍內的角度，如果路徑轉超過
%   180 度，phi 會從 +179° 突然跳到 -179°，直接微分會產生一個巨大的
%   假曲率尖峰；unwrap() 會把這種跳變「攤平」成連續遞增/遞減的角度
%   （例如 +179° 後面接 +181° 而不是 -179°），微分才會得到正確的曲率。
%
% 物理可行性截斷（kmax）：
%       kmax = tan(35°) / L1
%   這是「阿克曼轉向幾何」把最大前輪轉角 35°（車輛物理極限）換算成
%   最大可行曲率：轉彎半徑 R = L1 / tan(delta)，曲率 kappa = 1/R = tan(delta)/L1，
%   代入 delta_max=35° 即得 kmax。任何超過這個曲率的估計值都會被
%   截斷（clamp）到 ±kmax，避免後面 pure_pursuit_controller.m／
%   select_best_path.m 拿到「車輛實際上轉不出來」的曲率去做速度規劃
%   或可行性判斷。
%
% 平滑化（移動平均濾波）：
%   gradient() 這種一階差分對雜訊很敏感（尤其是路徑點間距不均勻、
%   或路徑本身有拼接痕跡時），所以最後用視窗寬度 win=9 的等權移動平均
%   （conv 搭配首尾值填充邊界，避免邊界效應）把曲率序列平滑化，
%   降低高頻雜訊對下游速度規劃與可行性判斷的影響。
%
% 輸入：
%   x, y   : Nx1 路徑點座標
%   phi    : Nx1 路徑點航向角（rad）
%   params : 需要 params.L1（拖車頭軸距，算 kmax 用）
%
% 輸出：
%   kappa : Nx1 平滑化、截斷後的曲率估計（1/m）
%
% 呼叫端：
%   my_multi_path.m（整條路線版）、generate_local_paths.m（即時局部版）
%   都用本函式從 stitch_local_path.m 拼出的路徑點算曲率。
% =========================================================================

function kappa = compute_path_curvature(x, y, phi, params)
    ds = hypot(gradient(x), gradient(y));   % 相鄰點間的弧長微元估計
    ds(ds<1e-6) = 1e-6;                      % 防止除以零（重複點的情況）
    kappa = gradient(unwrap(phi)) ./ ds;     % kappa ≈ d(theta)/ds
    kmax = tan(deg2rad(35)) / params.L1;     % 阿克曼幾何：最大轉角35°換算成最大曲率
    kappa = max(min(kappa, kmax), -kmax);    % 物理可行性截斷
    win = 9;                                 % 移動平均平滑視窗寬度
    kernel = ones(1,win)/win;
    % 首尾用邊界值填充，避免 conv 的 'valid' 模式在頭尾產生邊界效應
    kappa = conv([kappa(1)*ones(1,(win-1)/2), kappa(:)', kappa(end)*ones(1,(win-1)/2)], kernel, 'valid');
    kappa = kappa(:);
end
