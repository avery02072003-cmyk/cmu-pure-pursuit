% =========================================================================
% 檔案名稱: compute_hitch_speed_cap.m
%
% 功能：這次新增的模組。依路徑曲率 kappa，預先估計「如果照這個曲率走，
%       貨櫃穩態鉸接角會是多少」，再依此換算出一個「鉸接角限速」
%       v_hitch，跟既有的側向加速度限速 v_lat 一起送進
%       compute_v_profile.m 取最小值。這是「規劃階段」的第一道防線：
%       在車輛還沒真正轉進彎道之前，就先用曲率預估、提早降速，避免
%       事到臨頭鉸接角才逼近折疊上限。跟即時動態的第二道防線
%       hitch_angle_governor.m（用實際量到的鉸接角即時降速）互為
%       前後呼應，見該檔案檔頭說明。
%
% 數學推導（穩態圓周運動下，鉸接角與曲率的關係）：
%   假設拖車頭以固定曲率 kappa（半徑 R=1/kappa）做穩態圓周運動，鉸接點
%   跟著拖車頭走同一個圓；貨櫃後軸則因為被 L2 這段剛性連桿拖著走，
%   會沿著一個「半徑較小、圓心相同」的同心圓運動（這是聯結車轉彎必然
%   內切的幾何原因）。鉸接角 phi（拖車頭航向與貨櫃航向的夾角）在穩態
%   時滿足直角三角形關係：鉸接點到貨櫃後軸的連桿長度 L2、貨櫃圓周
%   半徑 R2，兩者與 phi 的關係近似為：
%
%       tan(phi_ss) ≈ L2 / R  =  L2 * kappa
%       →  phi_ss ≈ atan(L2 * kappa)
%
%   （這是小曲率半徑近似下的標準聯結車穩態轉彎幾何公式，也是
%   select_best_path.m 的 check_hitch_angle 用來預先判斷候選路徑
%   是否可行的同一個公式，兩處保持一致。）
%
%   ⚠ 為什麼不能用 asin(L2*kappa)（這是這次修正的一個關鍵 bug）：
%   本專案車輛 L2=7.5m，物理曲率上限 kmax=tan(35°)/L1≈0.156，
%   兩者相乘 L2*kmax≈1.17，「大於 1」。asin() 的定義域是 [-1,1]，
%   一旦 L2*kappa 超過 1，asin 就會飽和到接近 90°（且需要额外用
%   min(...,0.999) 去防止定義域外的 NaN），導致只要曲率稍微大一點，
%   預測出來的鉸接角就會被誇大到接近 90°，幾乎所有候選路徑都會被
%   誤判為「不可行」，被迫每次都退回置中路徑 —— 這正是舊版程式碼
%   選路機制形同虛設的根本原因之一（另一個原因是 my_multi_path.m
%   的轉置錯誤，見 stitch_local_path.m 說明）。改用 atan()
%   後，對任意大小的 L2*kappa 都能得到平滑、有界（趨近但不超過 90°）
%   的合理估計值，不會有這個飽和問題。
%
% 換算成限速（跟既有 a_lat_max -> v_lat 的做法同一套邏輯）：
%   由於 phi_ss 只跟 kappa 有關、跟速度 v 無關（穩態圓周運動的幾何
%   關係不含 v），所以嚴格來說「鉸接角」本身沒辦法直接換算出一個
%   隨速度變化的限速值。這裡採用的做法是：先算出這條路徑「最尖銳處」
%   預估會產生的鉸接角 predicted_hitch，跟安全門檻 margin（phi_max 的
%   一個比例）比較：
%       - 低於 margin：完全不限速（v_hitch = v_des，不影響速度規劃）
%       - 超過 margin：依「超過的比例」平滑降速（用一個一階遞減函數，
%         不是硬切斷，避免速度指令在門檻附近抖動）：
%
%           over = (predicted_hitch - margin) / (phi_max - margin)
%           v_hitch = v_des / (1 + gain * over)
%
%   over 越大（越接近甚至超過 phi_max），分母越大，v_hitch 就被壓得
%   越低。gain 越大，同樣的 over 值降速降得越兇。這不是嚴謹的動力學
%   推導（真正嚴謹要解聯結車的動態方程式），而是一個工程上「曲率越
%   靠近鉸接角上限、就越保守」的啟發式限速器，目的是在路徑規劃階段
%   先降低風險，實際安全底線仍由即時的 hitch_angle_governor.m 把關。
%
% 輸入：
%   kappa  : 路徑曲率序列（可以是純量或向量，逐點計算）
%   params : 需要 L2, phi_max, hitch_speed_cap_frac, hitch_speed_cap_gain, v_des
%
% 輸出：
%   v_hitch : 對應每個 kappa 值的鉸接角限速 (m/s)
% =========================================================================

function v_hitch = compute_hitch_speed_cap(kappa, params)
    predicted_hitch = atan(params.L2 * abs(kappa));            % 穩態鉸接角預估（見上方推導）
    margin = params.phi_max * params.hitch_speed_cap_frac;     % margin 以下完全不限速
    over = max(0, predicted_hitch - margin) ./ max(params.phi_max - margin, 1e-6);
    v_hitch = params.v_des ./ (1 + params.hitch_speed_cap_gain * over);  % 平滑降速（非硬切斷）
end
