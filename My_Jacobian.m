% =========================================================================
% 檔案名稱: My_Jacobian.m
%
% 功能：計算「終點狀態 [x_end; y_end; theta_end; kappa_end] 對曲率多項式
%       係數 [b, c, d, s_f] 的偏導數」，組成 4x4 Jacobian 矩陣，供
%       my_path.m 的 Newton-Raphson 迭代每一步用來修正係數猜測值。
%
% 為什麼需要 Jacobian（Newton-Raphson 的基本原理）：
%   my_path.m 要解一個非線性方程組：F(p) = endpoint(p) - target = 0，
%   其中 p=[b,c,d,s_f]，F 是「代入 p 後算出的終點狀態」與「目標狀態」的
%   差。Newton-Raphson 每次迭代用一階泰勒展開線性近似 F，
%   F(p+delta_p) ≈ F(p) + J*delta_p，令其為 0 解出修正量：
%       delta_p = -J^{-1} * F(p) = J^{-1} * (target - endpoint(p))
%   所以需要 J = dF/dp = d(endpoint)/dp 這個 4x4 矩陣，就是本函式要算的。
%
% 完整推導（座標系：曲率 kappa(s)=b*s+c*s^2+d*s^3，
%           航向 theta(s)=(1/2)b*s^2+(1/3)c*s^3+(1/4)d*s^4）：
%
%   終點 x_end、y_end 是積分（無解析解，數值算）：
%       x_end(p) = F_0 = ∫[0,s_f] cos(theta(s;b,c,d)) ds
%       y_end(p) = G_0 = ∫[0,s_f] sin(theta(s;b,c,d)) ds
%
%   對 b 微分（鏈式法則：d(cos(theta))/db = -sin(theta)*d(theta)/db，
%   而 d(theta(s))/db = s^2/2）：
%       d(x_end)/db = ∫[0,s_f] -sin(theta(s)) * (s^2/2) ds = -(1/2)*G_2
%       d(y_end)/db = ∫[0,s_f]  cos(theta(s)) * (s^2/2) ds =  (1/2)*F_2
%   同理對 c（d(theta)/dc = s^3/3）：
%       d(x_end)/dc = -(1/3)*G_3 ,   d(y_end)/dc = (1/3)*F_3
%   對 d（d(theta)/dd = s^4/4）：
%       d(x_end)/dd = -(1/4)*G_4 ,   d(y_end)/dd = (1/4)*F_4
%
%   把 (b,c,d) 這三欄統一寫成通式：對第 i 個係數（i=1→b, i=2→c, i=3→d，
%   對應冪次 z=i+1），
%       d(x_end)/dp(i) = -(1/z) * G_z ,   d(y_end)/dp(i) = (1/z) * F_z
%   這正是程式碼下面迴圈裡 Jacobian_M(1,i)=(-1/z)*Gk(z)、
%   Jacobian_M(2,i)=(1/z)*Fk(z) 的由來（Fk(z)、Gk(z) 由 my_Fkn_Gkn(z,...)
%   數值積分算出）。
%
%   對 s_f 微分則不需要數值積分 —— 利用「萊布尼茲積分法則」
%   （對變動上限的積分求導 = 被積函數在上限處的值）：
%       d(x_end)/d(s_f) = cos(theta(s_f)) = cos(theta_sf)
%       d(y_end)/d(s_f) = sin(theta(s_f)) = sin(theta_sf)
%   這就是程式碼裡 Jacobian_M(1,n)=cos(theda_sf)、
%   Jacobian_M(2,n)=sin(theda_sf) 的由來。
%
%   theta_end、kappa_end 本身就是解析多項式，偏導數直接逐項微分即可，
%   不需要用到 Fk/Gk：
%       theta_end = (1/2)b*s_f^2 + (1/3)c*s_f^3 + (1/4)d*s_f^4
%         → d/db = s_f^2/2, d/dc = s_f^3/3, d/dd = s_f^4/4
%           （同樣是 s_f^z/z 的通式，z=i+1）
%         → d/d(s_f) = b*s_f + c*s_f^2 + d*s_f^3 = kappa(s_f)
%           （萊布尼茲法則：theta(s_f)=∫kappa，對上限微分=被積函數本身）
%       kappa_end = b*s_f + c*s_f^2 + d*s_f^3
%         → d/db = s_f, d/dc = s_f^2, d/dd = s_f^3（通式 s_f^i）
%         → d/d(s_f) = b + 2c*s_f + 3d*s_f^2 = kappa'(s_f)（曲率對弧長的導數）
%
% 輸入：
%   col_num  : Jacobian 的欄數/係數個數 n（呼叫端固定用 4，對應 b,c,d,s_f）
%   part_num : 內部呼叫 my_Fkn_Gkn 數值積分用的取樣點數 k
%   p_ceff   : 目前這一輪迭代的曲率多項式係數 [b, c, d, s_f]
%
% 輸出：
%   output_args : 4x4 Jacobian 矩陣，列＝[x_end; y_end; theta_end; kappa_end]，
%                 欄＝對 [b; c; d; s_f] 的偏導數
% =========================================================================

function [ output_args ] = My_Jacobian( col_num, part_num, p_ceff )
n=col_num;
k=part_num;

p=p_ceff;
s_f=p(4);
s=linspace(0,s_f,k);

% 先把 F_1..F_n、G_1..G_n 全部算出來（i=1..4 對應冪次 1..4），
% 因為下面組 Jacobian 前三欄（對 b,c,d 微分）都要用到 F_2,F_3,F_4 / G_2,G_3,G_4
for i=1:n
 [ Fk(i) Gk(i)] = my_Fkn_Gkn(i, k, p);
end

J_M=zeros(4,n);   % （此變數後續未被使用，實際輸出矩陣是下面的 Jacobian_M；
                   %  保留原樣不刪除，避免影響任何既有呼叫端假設的行為）

% --- 前 n-1 欄：對曲率係數 (b, c, d) 的偏導數 ---
for i=1:n-1
z=i+1;
Jacobian_M(1,i)=(-1/z)*Gk(z);        % d(x_end)/dp(i) = -(1/z)*G_z
Jacobian_M(2,i)=(1/z)*Fk(z);         % d(y_end)/dp(i) =  (1/z)*F_z
Jacobian_M(3,i)=p(4)^(z)/z;          % d(theta_end)/dp(i) = s_f^z / z
Jacobian_M(4,i)=p(4)^(i);            % d(kappa_end)/dp(i) = s_f^i
z=0;
end

% theta(s_f)：終點航向角（等一下要用萊布尼茲法則算對 s_f 的偏導數）
theda_sf=(1/2)*p(1)*p(4)^2+(1/3)*p(2)*p(4)^3+(1/4)*p(3)*p(4)^4;

% --- 第 n 欄：對總弧長 s_f 的偏導數（萊布尼茲法則，不需數值積分）---
Jacobian_M(1,n)=cos(theda_sf);                          % d(x_end)/d(s_f)   = cos(theta(s_f))
Jacobian_M(2,n)=sin(theda_sf);                           % d(y_end)/d(s_f)   = sin(theta(s_f))
Jacobian_M(3,n)=p(1)*p(4)+p(2)*p(4)^2+p(3)*p(4)^3;        % d(theta_end)/d(s_f) = kappa(s_f)
Jacobian_M(4,n)=p(1)+2*p(2)*p(4)+3*p(3)*p(4)^2;           % d(kappa_end)/d(s_f) = kappa'(s_f)

JJJ=Jacobian_M;

output_args=JJJ;

end
