% =========================================================================
% 檔案名稱: my_Fkn_Gkn.m
%
% 功能：用「複合辛普森法則（Composite Simpson's Rule）」數值計算下面這兩個
%       帶權重的積分（權重是弧長的 power 次方）：
%
%           F_n = ∫[0, L] s^n * cos(theta(s)) ds
%           G_n = ∫[0, L] s^n * sin(theta(s)) ds
%
%       其中 theta(s) 是 my_fk_gk.m 算出來的瞬時航向角，L = p_ceff(4) = s_f
%       是這一段路徑的總弧長。
%
% 這兩個積分是整個路徑生成引擎的「數學心臟」，用途分兩種：
%   1. n=0 時：F_0 = 路徑終點 x 座標，G_0 = 路徑終點 y 座標
%      （my_endpoint.m 直接呼叫 n=0 版本取得終點位置）
%   2. n=1,2,3 時：F_n、G_n 是計算 Jacobian（終點狀態對曲率係數 b,c,d 的
%      偏導數）時會用到的中間量，因為對 theta(s) 裡的 b 微分會多產生一個
%      s^2 的因子、對 c 微分產生 s^3、對 d 微分產生 s^4（連鎖律），
%      所以 Jacobian 的每個元素都可以寫成「F_n 或 G_n 的線性組合」，
%      詳細推導見 My_Jacobian.m 檔頭註解。
%
% 為什麼要用數值積分（Simpson's rule）而不是解析解：
%   theta(s) 是 s 的四次多項式，所以 cos(theta(s))、sin(theta(s)) 是三角函數
%   套多項式，沒有初等函數的解析積分公式（跟菲涅耳積分 / 迴旋線
%   (clothoid) 積分同一類問題），只能用數值積分方法近似計算。
%   辛普森法則用二次多項式局部逼近被積函數，精度高、計算量小，
%   是這類問題的標準做法。
%
% 辛普森法則原理（複合型，k 個等距取樣點，k 需為偶數個區間即奇數個點）：
%   把 [0, L] 分成 (k-1) 個等寬區間，寬度 delta_s = L/(k-1)，在每個取樣點
%   s_i 上依照「1, 4, 2, 4, 2, ..., 4, 1」的權重加權求和，最後乘上 delta_s/3：
%
%       ∫[0,L] f(s) ds ≈ (delta_s/3) * [f(s_1) + 4f(s_2) + 2f(s_3) + 4f(s_4)
%                                         + ... + 4f(s_{k-1}) + f(s_k)]
%
%   本函式的權重表：奇數索引（i=1,3,5,...，對應數學上的偶數點，因為 MATLAB
%   索引從 1 開始）權重為 2，偶數索引權重為 4，兩端點（i=1 和 i=k）權重為 1
%   （程式碼裡先用迴圈全部填成 2/4，跑完迴圈後再把頭尾覆寫成 1，等效於
%   標準辛普森權重表）。要帶 power 次方權重 s^n 時，直接把 s^n 乘進被積
%   函數（f_k, g_k）裡即可，因為 s^n 是已知函數、跟辛普森積分法本身無關。
%
% 輸入：
%   power       : 冪次 n（積分裡 s^n 的 n）
%   part_number : 辛普森積分的取樣點數 k（呼叫端固定用 k=20）
%   p_ceff      : 曲率多項式係數向量 [b, c, d, s_f]
%
% 輸出：
%   output_args1 : F_n = ∫[0,L] s^n * cos(theta(s)) ds
%   output_args2 : G_n = ∫[0,L] s^n * sin(theta(s)) ds
% =========================================================================

function [ output_args1 output_args2 ] = my_Fkn_Gkn(power, part_number, p_ceff)
p=p_ceff;
n=power;
k=part_number;
L=p(4);                    % 積分上限 = 這一段路徑的總弧長 s_f
s=linspace(0,L,k);         % k 個等距弧長取樣點，落在 [0, L] 之間
delta_s=s(2)-s(1);         % 相鄰取樣點間距（辛普森公式的 h）

for i=1:k
    % 依索引奇偶決定辛普森權重：偶數索引（z==0，即 i=2,4,6,...）權重 4，
    % 奇數索引權重 2（頭尾兩個端點稍後會被覆寫成 1，見迴圈後方）
    z=rem(i,2);
    [ f_k(i) g_k(i) ] = my_fk_gk(p,s(i));   % 被積函數本體：cos(theta(s_i))、sin(theta(s_i))
    if z==0
        w(i)=4;
        f_Temp(i)=w(i)*f_k(i)*s(i)^n;       % 乘上權重 w(i) 與冪次項 s_i^n
        g_Temp(i)=w(i)*g_k(i)*s(i)^n;
    else
        w(i)=2;
        f_Temp(i)=w(i)*f_k(i)*s(i)^n;
        g_Temp(i)=w(i)*g_k(i)*s(i)^n;
    end
end

% 覆寫頭尾兩端點權重為 1（標準複合辛普森法則的邊界權重）
        w(1)=1;
        f_Temp(1)=w(1)*f_k(1)*s(1)^n;
        g_Temp(1)=w(1)*g_k(1)*s(1)^n;
        w(k)=1;
        f_Temp(k)=w(k)*f_k(k)*s(k)^n;
        g_Temp(k)=w(k)*g_k(k)*s(k)^n;

       f_Temp2=sum(f_Temp);
       g_Temp2=sum(g_Temp);
       % 辛普森法則最終公式：(delta_s/3) * 加權和
       output_args1=f_Temp2*(delta_s/3);   % F_n
       output_args2=g_Temp2*(delta_s/3);   % G_n

end
