% =========================================================================
% 檔案名稱: shape_v_profile_cubic.m
%
% 功能：把 compute_v_profile.m 算出來的「安全速度上限」剖面，用三次
%       多項式（以弧長 s 為自變量、速度 v 為因變量）重新整形成一條
%       「速度、加速度在起點都跟車輛目前真實狀態連續銜接」的平滑減速
%       曲線，直接覆寫進即時控制實際使用的 refpath_active.v_profile。
%
% 緣由（移植自舊專案 論文2/cmu path method/my_velocity.m、
% Untitled3_path_velocity_genarator.m 的核心數學，重新推導、換了變數名、
% 補上完整推導，不是逐字複製舊檔案）：
%       compute_v_profile.m 的 backward/forward pass 是逐點 min() 夾出
%       一條「安全上限」曲線，物理上正確（每一段都滿足 sqrt(v²+2a·ds)
%       的等加速度運動學公式），但因為是逐點夾出來的，在「側向加速度
%       限速」跟「減速限速」兩種不同約束交替生效的地方，加速度本身
%       (dv/ds 的斜率) 可能不連續——這對車輛物理上仍然可行（安全上限
%       只保證「不超過」），但不是最平滑、最像真實駕駛的減速方式。
%       三次多項式弧長速度剖面（形式跟 my_path.m 用三次曲率多項式產生
%       平滑路徑幾何是同一種思路，只是這裡用在速度而非曲率上）可以
%       直接指定起點/終點的速度「跟」加速度邊界條件，解出一條 C1
%       連續（速度、加速度都連續）的曲線，銜接處不會有加速度跳變。
%
% 數學推導：
%       給定邊界條件 s=0 時 v=v0, dv/ds=a0；s=s1 時 v=v1, dv/ds=a1，
%       用三次多項式 v(s) = q1 + q2*s + q3*s^2 + q4*s^3 內插。
%       四個邊界條件解四個未知係數：
%           v(0)=q1=v0                                    → q1=v0
%           v'(0)=q2=a0                                    → q2=a0
%           v(s1)=q1+q2*s1+q3*s1^2+q4*s1^3=v1
%           v'(s1)=q2+2*q3*s1+3*q4*s1^2=a1
%       解這組聯立方程式（跟三次 Hermite 插值同一套代數）得：
%           q3 = -2*a0/s1 - a1/s1 - 3*v0/s1^2 + 3*v1/s1^2
%           q4 =  a0/s1^2 + a1/s1^2 + 2*v0/s1^3 - 2*v1/s1^3
%       （可代回 v(s1)、v'(s1) 驗證恆成立，這就是 my_velocity.m 裡
%       q(3)、q(4) 公式的來源，這裡原樣保留、只是換了容易讀的變數名。）
%
% 邊界條件在這個系統裡怎麼決定：
%   v0, a0 : 車輛「當下真實」的速度與縱向加速度（呼叫端傳入，不是
%            v_profile(1)——用真實狀態才能確保跟上一次 replan 銜接處
%            連續，不會每次 replan 都有一個加速度跳變）
%   v1     : 這個候選路徑（約 local_horizon_m=35m 窗口）安全速度剖面
%            的全域最小值——也就是這段窗口裡「最該配合減速」的那一點，
%            通常對應窗口內最緊的彎道
%   s1     : v1 所在的弧長位置
%   a1     : 固定取 0，假設車輛到達那個最緊的點時速度已經穩定，
%            不再加減速（跟真實駕駛「進彎前煞車、彎中維持等速」的
%            直覺一致）
%
% 安全性保證（這是本函式最重要的設計原則）：
%       三次多項式解出來的曲線只能拿來把 s<=s1 這段「壓低」到既有安全
%       上限之下（v_profile_shaped = min(v_cubic, v_profile_envelope)），
%       不能超過既有安全上限。也就是說，即使 a0 的估計有誤差、或邊界
%       條件不理想，最差情況只是曲線不夠平滑（跟原本的安全上限一樣），
%       絕對不會比 compute_v_profile.m 算出來的物理安全上限更快、更危險。
%
% 退化保護：
%       如果 s1 太靠近窗口起點（該最小值幾乎就在車輛腳下，例如車輛已經
%       身處彎道中），三次多項式在極短距離內解出來的曲線容易病態（q3、
%       q4 分母趨近於零、數值爆炸），這種情況直接跳過整形、原樣回傳
%       既有安全剖面。
%
% 輸入：
%   refpath_active : 候選路徑 struct，需要 .x .y .v_profile
%                     （.v_profile 是 compute_v_profile.m 算出的安全上限）
%   v_now          : 車輛當下真實速度 (m/s)
%   a_now          : 車輛當下估計的縱向加速度 (m/s²)，呼叫端用有限差分
%                     (v-v_prev_step)/Ts 估計
%   params         : 需要 v_profile_min, v_des（最終安全鉗制範圍）
%
% 輸出：
%   v_profile_shaped : Nx1，整形後的速度剖面，永遠 <= 原本的
%                       refpath_active.v_profile（見上方安全性保證）
%
% 呼叫端：main_pure_pursuit_sim.m（每次 replan 之後，覆寫
%         refpath_active.v_profile，讓即時控制迴圈讀到的
%         v_ref_now = refpath_active.v_profile(idx_near) 真的用到這條
%         平滑曲線，不是只是額外畫出來好看的參考線）
% =========================================================================

function v_profile_shaped = shape_v_profile_cubic(refpath_active, v_now, a_now, params)
    v_profile_shaped = refpath_active.v_profile;   % 預設：不整形，原樣回傳既有安全上限

    s = [0; cumsum(hypot(diff(refpath_active.x), diff(refpath_active.y)))];
    v_env = refpath_active.v_profile;

    [v1, idx1] = min(v_env);
    s1 = s(idx1);

    % 退化保護：最緊的點幾乎就在窗口起點，三次多項式在極短距離內解會病態，跳過整形
    if idx1 <= 2 || s1 < 1e-3
        return;
    end

    v0 = v_now;
    a0 = a_now;
    a1 = 0;   % 假設到達最緊的點時已經穩定，不再加減速

    q1 = v0;
    q2 = a0;
    q3 = -2*a0/s1 - a1/s1 - 3*v0/s1^2 + 3*v1/s1^2;
    q4 =  a0/s1^2 + a1/s1^2 + 2*v0/s1^3 - 2*v1/s1^3;

    in_ramp = s <= s1;
    s_ramp = s(in_ramp);
    v_cubic = q1 + q2*s_ramp + q3*s_ramp.^2 + q4*s_ramp.^3;

    % 安全性保證：整形後的曲線不能超過既有安全上限，只能把它壓平滑，見檔頭說明
    v_profile_shaped(in_ramp) = min(v_cubic, v_env(in_ramp));

    % 最終安全鉗制，防止三次多項式在邊界條件之間解出超出範圍的內部極值
    v_profile_shaped = max(params.v_profile_min, min(v_profile_shaped, params.v_des));
end
